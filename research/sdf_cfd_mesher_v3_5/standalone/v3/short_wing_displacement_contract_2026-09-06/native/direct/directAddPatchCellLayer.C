/*---------------------------------------------------------------------------*\
  Direct research adapter for the stock OpenFOAM Foundation 13
  Foam::addPatchCellLayer topology kernel.

  This adapter supplies data and records ancestry. It does not create or
  modify layer topology itself.
\*---------------------------------------------------------------------------*/

#include "argList.H"
#include "Time.H"
#include "polyMesh.H"
#include "fvMesh.H"
#include "IOdictionary.H"
#include "addPatchCellLayer.H"
#include "polyTopoChange.H"
#include "polyTopoChangeMap.H"
#include "indirectPrimitivePatch.H"
#include "globalIndex.H"
#include "IndirectList.H"
#include "ListOps.H"
#include "OFstream.H"
#include "OSspecific.H"

using namespace Foam;


int main(int argc, char *argv[])
{
    argList::addNote
    (
        "Create a stand-alone adaptive layer region through the stock "
        "Foundation 13 addPatchCellLayer kernel"
    );

    #include "setRootCase.H"
    #include "createTime.H"

    polyMesh sourceMesh
    (
        IOobject
        (
            polyMesh::defaultRegion,
            runTime.constant(),
            runTime,
            IOobject::MUST_READ
        )
    );

    IOdictionary dict
    (
        IOobject
        (
            "directAddPatchCellLayerDict",
            runTime.system(),
            sourceMesh,
            IOobject::MUST_READ,
            IOobject::NO_WRITE
        )
    );

    const wordList sourcePatches(dict.lookup("sourcePatches"));
    const word exposedPatchName(dict.lookup("exposedPatchName"));
    const word outputRegion
    (
        dict.lookupOrDefault<word>("outputRegion", "layerRegion")
    );

    const labelList meshFaceLayers(dict.lookup("meshFaceLayers"));
    const labelList meshPointLayers(dict.lookup("meshPointLayers"));
    const scalarField meshPointExpansionRatio
    (
        dict.lookup("meshPointExpansionRatio")
    );
    const vectorField meshPointFirstLayerDisp
    (
        dict.lookup("meshPointFirstLayerDisp")
    );

    if
    (
        meshFaceLayers.size() != sourceMesh.nFaces()
     || meshPointLayers.size() != sourceMesh.nPoints()
     || meshPointExpansionRatio.size() != sourceMesh.nPoints()
     || meshPointFirstLayerDisp.size() != sourceMesh.nPoints()
    )
    {
        FatalErrorInFunction
            << "Input lists must be indexed over the complete source mesh." << nl
            << "mesh faces=" << sourceMesh.nFaces()
            << " face layers=" << meshFaceLayers.size() << nl
            << "mesh points=" << sourceMesh.nPoints()
            << " point layers=" << meshPointLayers.size()
            << " ratios=" << meshPointExpansionRatio.size()
            << " first displacements=" << meshPointFirstLayerDisp.size()
            << exit(FatalError);
    }

    DynamicList<label> selectedFaces;
    const auto patchIndex = [&](const word& name)
    {
        forAll(sourceMesh.boundaryMesh(), patchi)
        {
            if (sourceMesh.boundaryMesh()[patchi].name() == name)
            {
                return patchi;
            }
        }
        return label(-1);
    };

    forAll(sourcePatches, sourcePatchi)
    {
        const label patchi = patchIndex(sourcePatches[sourcePatchi]);

        if (patchi < 0)
        {
            FatalErrorInFunction
                << "Cannot find source patch " << sourcePatches[sourcePatchi]
                << exit(FatalError);
        }

        const polyPatch& patch = sourceMesh.boundaryMesh()[patchi];
        forAll(patch, localFacei)
        {
            selectedFaces.append(patch.start() + localFacei);
        }
    }

    const labelList patchFaces(selectedFaces);
    if (patchFaces.empty())
    {
        FatalErrorInFunction << "No source faces selected" << exit(FatalError);
    }

    indirectPrimitivePatch extrudePatch
    (
        IndirectList<face>(sourceMesh.faces(), patchFaces),
        sourceMesh.points()
    );

    const labelList& patchMeshPoints = extrudePatch.meshPoints();

    labelList nFaceLayers(extrudePatch.size());
    forAll(nFaceLayers, patchFacei)
    {
        nFaceLayers[patchFacei] = meshFaceLayers[patchFaces[patchFacei]];
    }

    labelList nPointLayers(extrudePatch.nPoints());
    scalarField expansionRatio(extrudePatch.nPoints());
    vectorField firstLayerDisp(extrudePatch.nPoints());
    forAll(patchMeshPoints, patchPointi)
    {
        const label meshPointi = patchMeshPoints[patchPointi];
        nPointLayers[patchPointi] = meshPointLayers[meshPointi];
        expansionRatio[patchPointi] = meshPointExpansionRatio[meshPointi];
        firstLayerDisp[patchPointi] = meshPointFirstLayerDisp[meshPointi];
    }

    const globalIndex globalFaces(sourceMesh.nFaces());
    const labelListList edgeGlobalFaces
    (
        addPatchCellLayer::globalEdgeFaces
        (
            sourceMesh,
            globalFaces,
            extrudePatch
        )
    );

    labelList sidePatchID;
    label nPatches;
    Map<label> nbrProcToPatch;
    Map<label> patchToNbrProc;
    addPatchCellLayer::calcSidePatch
    (
        sourceMesh,
        globalFaces,
        edgeGlobalFaces,
        extrudePatch,
        sidePatchID,
        nPatches,
        nbrProcToPatch,
        patchToNbrProc
    );

    if (nPatches != sourceMesh.boundaryMesh().size())
    {
        FatalErrorInFunction
            << "This serial research adapter does not add processor patches. "
            << "source patches=" << sourceMesh.boundaryMesh().size()
            << " required patches=" << nPatches
            << exit(FatalError);
    }

    const label exposedPatchi = patchIndex(exposedPatchName);
    if (exposedPatchi < 0)
    {
        FatalErrorInFunction
            << "Cannot find exposed patch " << exposedPatchName
            << exit(FatalError);
    }
    const labelList exposedPatchID(extrudePatch.size(), exposedPatchi);
    const labelList faceCellZones(extrudePatch.size(), -1);

    addPatchCellLayer layerExtrude(sourceMesh, false);
    polyTopoChange meshMod(sourceMesh.boundaryMesh().size());

    layerExtrude.setRefinement
    (
        sourceMesh,
        globalFaces,
        edgeGlobalFaces,
        expansionRatio,
        extrudePatch,
        sidePatchID,
        exposedPatchID,
        nFaceLayers,
        nPointLayers,
        firstLayerDisp,
        faceCellZones,
        meshMod
    );

    autoPtr<fvMesh> layerMesh;
    autoPtr<polyTopoChangeMap> map = meshMod.makeMesh
    (
        layerMesh,
        IOobject
        (
            outputRegion,
            runTime.constant(),
            runTime,
            IOobject::NO_READ,
            IOobject::AUTO_WRITE,
            false
        ),
        sourceMesh
    );

    layerExtrude.updateZones(layerMesh());
    layerMesh().topoChange(map());
    layerExtrude.topoChange
    (
        map(),
        identityMap(extrudePatch.size()),
        identityMap(extrudePatch.nPoints())
    );

    // In stand-alone mode the first layer face is a boundary face, so the
    // addPatchCellLayer::addedCells helper (which reads faceNeighbour) is not
    // applicable. The stock kernel creates cells in patch-face/layer order.
    // Record that insertion ancestry explicitly and validate it downstream.
    labelListList addedCells(nFaceLayers.size());
    label nextCell = 0;
    forAll(nFaceLayers, patchFacei)
    {
        addedCells[patchFacei].setSize(nFaceLayers[patchFacei]);
        forAll(addedCells[patchFacei], layeri)
        {
            addedCells[patchFacei][layeri] = nextCell++;
        }
    }

    if (nextCell != layerMesh().nCells())
    {
        FatalErrorInFunction
            << "Unexpected stand-alone cell count. Recorded " << nextCell
            << " but mesh has " << layerMesh().nCells()
            << exit(FatalError);
    }

    layerMesh().write();

    const fileName evidenceDir(runTime.path()/"postProcessing"/"directLayer");
    mkDir(evidenceDir);
    OFstream ancestry(evidenceDir/"ancestry");
    ancestry
        << "sourcePatchFaces " << patchFaces << token::END_STATEMENT << nl
        << "sourcePatchMeshPoints " << patchMeshPoints
        << token::END_STATEMENT << nl
        << "requestedFaceLayers " << nFaceLayers
        << token::END_STATEMENT << nl
        << "requestedPointLayers " << nPointLayers
        << token::END_STATEMENT << nl
        << "expansionRatio " << expansionRatio
        << token::END_STATEMENT << nl
        << "firstLayerDisp " << firstLayerDisp
        << token::END_STATEMENT << nl
        << "addedPoints " << layerExtrude.addedPoints()
        << token::END_STATEMENT << nl
        << "layerFaces " << layerExtrude.layerFaces()
        << token::END_STATEMENT << nl
        << "addedCells " << addedCells
        << token::END_STATEMENT << nl;

    OFstream pointCsv(evidenceDir/"pointAncestry.csv");
    pointCsv
        << "patchPoint,sourceMeshPoint,baseOutputPoint,nPointLayers,ratio,"
        << "dx,dy,dz,addedPointIds" << nl;
    forAll(patchMeshPoints, patchPointi)
    {
        pointCsv
            << patchPointi << ','
            << patchMeshPoints[patchPointi] << ','
            << patchPointi << ','
            << nPointLayers[patchPointi] << ','
            << expansionRatio[patchPointi] << ','
            << firstLayerDisp[patchPointi].x() << ','
            << firstLayerDisp[patchPointi].y() << ','
            << firstLayerDisp[patchPointi].z() << ',';
        const labelList& ids = layerExtrude.addedPoints()[patchPointi];
        forAll(ids, i)
        {
            if (i) pointCsv << '|';
            pointCsv << ids[i];
        }
        pointCsv << nl;
    }

    OFstream faceCsv(evidenceDir/"faceAncestry.csv");
    faceCsv
        << "patchFace,sourceMeshFace,nFaceLayers,layerFaceIds,addedCellIds"
        << nl;
    forAll(patchFaces, patchFacei)
    {
        faceCsv
            << patchFacei << ','
            << patchFaces[patchFacei] << ','
            << nFaceLayers[patchFacei] << ',';
        const labelList& faces = layerExtrude.layerFaces()[patchFacei];
        forAll(faces, i)
        {
            if (i) faceCsv << '|';
            faceCsv << faces[i];
        }
        faceCsv << ',';
        const labelList& cells = addedCells[patchFacei];
        forAll(cells, i)
        {
            if (i) faceCsv << '|';
            faceCsv << cells[i];
        }
        faceCsv << nl;
    }

    Info<< "Created stand-alone layer mesh in region " << outputRegion << nl
        << "points=" << layerMesh().nPoints()
        << " faces=" << layerMesh().nFaces()
        << " cells=" << layerMesh().nCells() << nl
        << "ancestry=" << ancestry.name() << nl
        << endl;

    return 0;
}
