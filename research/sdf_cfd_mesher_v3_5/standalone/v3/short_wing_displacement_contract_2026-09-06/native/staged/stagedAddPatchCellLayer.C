/*---------------------------------------------------------------------------*\
  Research adapter for repeated one-physical-layer insertion through the
  stock OpenFOAM Foundation 13 Foam::addPatchCellLayer topology kernel.

  The adapter supplies displacement data and records persistent ancestry.
  It does not create layer topology.
\*---------------------------------------------------------------------------*/

#include "argList.H"
#include "Time.H"
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
#include "contractDump.H"


static label patchIndex(const polyMesh& mesh, const word& name)
{
    forAll(mesh.boundaryMesh(), patchi)
    {
        if (mesh.boundaryMesh()[patchi].name() == name)
        {
            return patchi;
        }
    }
    return -1;
}


int main(int argc, char *argv[])
{
    argList::addNote
    (
        "Add independent physical layers one at a time through the stock "
        "Foundation 13 addPatchCellLayer kernel"
    );

    #include "setRootCase.H"
    #include "createTime.H"

    fvMesh mesh
    (
        IOobject
        (
            fvMesh::defaultRegion,
            runTime.constant(),
            runTime,
            IOobject::MUST_READ
        )
    );

    IOdictionary dict
    (
        IOobject
        (
            "stagedAddPatchCellLayerDict",
            runTime.system(),
            mesh,
            IOobject::MUST_READ,
            IOobject::NO_WRITE
        )
    );

    const word sourcePatchName(dict.lookup("sourcePatch"));
    const List<vectorField> stageDisplacements(dict.lookup("stageDisplacements"));
    const List<vectorField> expectedFronts(dict.lookup("expectedFronts"));
    const List<labelList> sourceFaces(dict.lookup("sourceFaces"));
    const scalar coordinateTolerance(readScalar(dict.lookup("coordinateTolerance")));
    const label sourceVertexCount=expectedFronts[0].size();
    const label initialPointCount = mesh.nPoints();
    labelList pointAncestor(dict.lookup("initialPointAncestors"));
    labelList pointLevel(dict.lookup("initialPointLevels"));
    labelList cellAncestor(dict.lookup("initialCellAncestors"));
    labelList cellLevel(mesh.nCells(),1);
    const labelList initialPatchFaceAncestors(dict.lookup("initialPatchFaceAncestors"));
    if (stageDisplacements.size()+1!=expectedFronts.size()
     || pointAncestor.size()!=mesh.nPoints() || pointLevel.size()!=mesh.nPoints()
     || cellAncestor.size()!=mesh.nCells()) FatalErrorInFunction<<"InvalidContractShape"<<exit(FatalError);
    forAll(stageDisplacements,k) if(stageDisplacements[k].size()!=sourceVertexCount)
        FatalErrorInFunction<<"InvalidDisplacementShape"<<exit(FatalError);
    forAll(expectedFronts,k) if(expectedFronts[k].size()!=sourceVertexCount)
        FatalErrorInFunction<<"InvalidExpectedShape"<<exit(FatalError);
    forAll(pointAncestor,i) {
      if(pointAncestor[i]<0 || pointAncestor[i]>=sourceVertexCount || pointLevel[i]<0 || pointLevel[i]>1)
        FatalErrorInFunction<<"InvalidInitialPointAuthority"<<exit(FatalError);
      if(mag(mesh.points()[i]-expectedFronts[pointLevel[i]][pointAncestor[i]])>coordinateTolerance)
        FatalErrorInFunction<<"InitialCoordinateMismatch"<<exit(FatalError);
    }

    // The first current boundary patch establishes persistent face ancestry.
    label sourcePatchi = patchIndex(mesh, sourcePatchName);
    if (sourcePatchi < 0)
    {
        FatalErrorInFunction
            << "Cannot find source patch " << sourcePatchName
            << exit(FatalError);
    }

    labelList faceAncestor(mesh.nFaces(), -1);
    {
        const polyPatch& patch = mesh.boundaryMesh()[sourcePatchi];
        forAll(patch, localFacei)
        {
            faceAncestor[patch.start() + localFacei] = initialPatchFaceAncestors[localFacei];
        }
    }

    const fileName evidenceDir
    (
        runTime.path()/"postProcessing"/"stagedLayer"
    );
    mkDir(evidenceDir);

    OFstream stageCsv(evidenceDir/"stages.csv");
    stageCsv
        << "stage,patchPoint,currentMeshPoint,persistentPoint,requestedThickness,"
        << "dx,dy,dz,addedPoint,actualDx,actualDy,actualDz" << nl;

    OFstream faceCsv(evidenceDir/"faces.csv");
    faceCsv
        << "stage,patchFace,currentMeshFace,persistentFace,addedCell,outerFace"
        << nl;

    DynamicList<label> allAddedCells;

    for (label stagei=1;stagei<stageDisplacements.size();++stagei)
    {
        sourcePatchi = patchIndex(mesh, sourcePatchName);
        if (sourcePatchi < 0)
        {
            FatalErrorInFunction
                << "Cannot find source patch " << sourcePatchName
                << " at stage " << stagei << exit(FatalError);
        }

        const polyPatch& sourcePatch = mesh.boundaryMesh()[sourcePatchi];
        labelList patchFaces(sourcePatch.size());
        forAll(sourcePatch, localFacei)
        {
            patchFaces[localFacei] = sourcePatch.start() + localFacei;
        }

        indirectPrimitivePatch extrudePatch
        (
            IndirectList<face>(mesh.faces(), patchFaces),
            mesh.points()
        );

        const labelList patchMeshPoints(extrudePatch.meshPoints());
        labelList nFaceLayers(extrudePatch.size(), 1);
        labelList nPointLayers(extrudePatch.nPoints(), 1);
        scalarField expansionRatio(extrudePatch.nPoints(), 1.0);
        vectorField firstLayerDisp(extrudePatch.nPoints(), vector::zero);

        labelList patchPointAncestor(extrudePatch.nPoints(), -1);
        labelList patchFaceAncestor(extrudePatch.size(), -1);
        pointField oldPatchPoints(extrudePatch.nPoints());

        forAll(patchMeshPoints, patchPointi)
        {
            const label meshPointi = patchMeshPoints[patchPointi];
            const label anc = pointAncestor[meshPointi];
            if (anc < 0 || anc >= sourceVertexCount)
            {
                FatalErrorInFunction
                    << "Missing persistent point ancestry at stage " << stagei
                    << ", patch point " << patchPointi
                    << ", mesh point " << meshPointi << exit(FatalError);
            }
            patchPointAncestor[patchPointi] = anc;
            oldPatchPoints[patchPointi] = mesh.points()[meshPointi];
            firstLayerDisp[patchPointi] =
                stageDisplacements[stagei][anc];
            if(pointLevel[meshPointi]!=stagei) FatalErrorInFunction<<"FrontLevelMismatch"<<exit(FatalError);
        }

        forAll(patchFaces, patchFacei)
        {
            const label anc = faceAncestor[patchFaces[patchFacei]];
            if (anc < 0)
            {
                FatalErrorInFunction
                    << "Missing persistent face ancestry at stage " << stagei
                    << ", patch face " << patchFacei
                    << ", mesh face " << patchFaces[patchFacei]
                    << exit(FatalError);
            }
            patchFaceAncestor[patchFacei] = anc;
        }

        const globalIndex globalFaces(mesh.nFaces());
        const labelListList edgeGlobalFaces
        (
            addPatchCellLayer::globalEdgeFaces
            (
                mesh,
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
            mesh,
            globalFaces,
            edgeGlobalFaces,
            extrudePatch,
            sidePatchID,
            nPatches,
            nbrProcToPatch,
            patchToNbrProc
        );

        if (nPatches != mesh.boundaryMesh().size())
        {
            FatalErrorInFunction
                << "This serial research adapter does not add processor "
                << "patches. Current patches=" << mesh.boundaryMesh().size()
                << ", required patches=" << nPatches << exit(FatalError);
        }

        addPatchCellLayer layerExtrude(mesh, true);
        polyTopoChange meshMod(mesh);
        layerExtrude.setRefinement
        (
            mesh,
            globalFaces,
            edgeGlobalFaces,
            expansionRatio,
            extrudePatch,
            sidePatchID,
            labelList(0),
            nFaceLayers,
            nPointLayers,
            firstLayerDisp,
            labelList::null(),
            meshMod
        );

        const pointField oldAllPoints(mesh.points());
        const labelList oldPointLevel(pointLevel), oldCellAncestor(cellAncestor), oldCellLevel(cellLevel);
        const labelList oldPointAncestor(pointAncestor);
        const labelList oldFaceAncestor(faceAncestor);

        autoPtr<polyTopoChangeMap> map = meshMod.changeMesh(mesh);
        layerExtrude.updateZones(mesh);
        mesh.topoChange(map());
        layerExtrude.topoChange
        (
            map(),
            identityMap(extrudePatch.size()),
            identityMap(extrudePatch.nPoints())
        );

        labelList nextPointAncestor(mesh.nPoints(), -1), nextPointLevel(mesh.nPoints(),-1);
        const labelList& reversePointMap = map().reversePointMap();
        for (label oldPointi=0;oldPointi<oldPointAncestor.size();++oldPointi)
        {
            const label newPointi = reversePointMap[oldPointi];
            if (newPointi >= 0)
            {
                nextPointAncestor[newPointi] = oldPointAncestor[oldPointi];
                nextPointLevel[newPointi] = oldPointLevel[oldPointi];
                for(direction j=0;j<3;++j) if(mesh.points()[newPointi][j]!=oldAllPoints[oldPointi][j])
                    FatalErrorInFunction<<"CommittedPointMoved old="<<oldPointi<<" mapped="<<newPointi<<" priorCount="<<oldAllPoints.size()<<" reverseMapCount="<<reversePointMap.size()<<" old="<<oldAllPoints[oldPointi]<<" new="<<mesh.points()[newPointi]<<exit(FatalError);
            }
        }

        const labelListList& addedPoints = layerExtrude.addedPoints();
        forAll(addedPoints, patchPointi)
        {
            if (addedPoints[patchPointi].size() != 1)
            {
                FatalErrorInFunction
                    << "Stage " << stagei << " did not create exactly one "
                    << "point level at patch point " << patchPointi
                    << exit(FatalError);
            }
            const label newPointi = addedPoints[patchPointi][0];
            nextPointAncestor[newPointi] = patchPointAncestor[patchPointi];
            nextPointLevel[newPointi]=stagei+1;
            if(mag(mesh.points()[newPointi]-expectedFronts[stagei+1][patchPointAncestor[patchPointi]])>coordinateTolerance)
                FatalErrorInFunction<<"AddedCoordinateMismatch"<<exit(FatalError);

            const vector actualDisp =
                mesh.points()[newPointi] - oldPatchPoints[patchPointi];
            stageCsv
                << stagei + 1 << ',' << patchPointi << ','
                << patchMeshPoints[patchPointi] << ','
                << patchPointAncestor[patchPointi] << ','
                << mag(stageDisplacements[stagei][patchPointAncestor[patchPointi]])
                << ',' << firstLayerDisp[patchPointi].x()
                << ',' << firstLayerDisp[patchPointi].y()
                << ',' << firstLayerDisp[patchPointi].z()
                << ',' << newPointi
                << ',' << actualDisp.x()
                << ',' << actualDisp.y()
                << ',' << actualDisp.z() << nl;
        }
        pointAncestor.transfer(nextPointAncestor);
        pointLevel.transfer(nextPointLevel);

        labelList nextFaceAncestor(mesh.nFaces(), -1);
        const labelList& reverseFaceMap = map().reverseFaceMap();
        for (label oldFacei=0;oldFacei<oldFaceAncestor.size();++oldFacei)
        {
            const label newFacei = reverseFaceMap[oldFacei];
            if (newFacei >= 0 && oldFaceAncestor[oldFacei] >= 0)
            {
                nextFaceAncestor[newFacei] = oldFaceAncestor[oldFacei];
            }
        }

        labelList nextCellAncestor(mesh.nCells(),-1),nextCellLevel(mesh.nCells(),-1);
        const labelList& reverseCellMap=map().reverseCellMap();
        for(label oldi=0;oldi<oldCellAncestor.size();++oldi) if(reverseCellMap[oldi]>=0) {
          nextCellAncestor[reverseCellMap[oldi]]=oldCellAncestor[oldi];
          nextCellLevel[reverseCellMap[oldi]]=oldCellLevel[oldi];
        }
        const labelListList addedCells(layerExtrude.addedCells(mesh));
        const labelListList& layerFaces = layerExtrude.layerFaces();
        forAll(layerFaces, patchFacei)
        {
            if (layerFaces[patchFacei].size() != 2)
            {
                FatalErrorInFunction
                    << "Stage " << stagei << " did not create exactly one "
                    << "cell layer at patch face " << patchFacei
                    << exit(FatalError);
            }
            if (addedCells[patchFacei].size() != 1)
            {
                FatalErrorInFunction
                    << "Cannot recover the added cell at stage " << stagei
                    << ", patch face " << patchFacei << exit(FatalError);
            }

            const label outerFacei = layerFaces[patchFacei][1];
            nextFaceAncestor[outerFacei] = patchFaceAncestor[patchFacei];
            allAddedCells.append(addedCells[patchFacei][0]);
            nextCellAncestor[addedCells[patchFacei][0]]=patchFaceAncestor[patchFacei];
            nextCellLevel[addedCells[patchFacei][0]]=stagei+1;
            faceCsv
                << stagei + 1 << ',' << patchFacei << ','
                << patchFaces[patchFacei] << ','
                << patchFaceAncestor[patchFacei] << ','
                << addedCells[patchFacei][0] << ','
                << outerFacei << nl;
        }
        faceAncestor.transfer(nextFaceAncestor);
        cellAncestor.transfer(nextCellAncestor);cellLevel.transfer(nextCellLevel);
        const label k=stagei+1, F=sourceFaces.size(), E=3*F/2;
        if(mesh.nPoints()!=(k+1)*sourceVertexCount || mesh.nCells()!=k*F
          || mesh.nInternalFaces()!=k*E+(k-1)*F || mesh.nFaces()!=k*E+(k+1)*F)
          FatalErrorInFunction<<"SharedConnectivityCountMismatch"<<exit(FatalError);
        const labelListList& cp=mesh.cellPoints();
        forAll(cp,ci) {
          if(cp[ci].size()!=6 || cellAncestor[ci]<0 || cellLevel[ci]<1)
            FatalErrorInFunction<<"CellAncestryMismatch"<<exit(FatalError);
          label lower=0,upper=0;
          forAll(cp[ci],j) {
            label pi=cp[ci][j],anc=pointAncestor[pi];
            if(findIndex(sourceFaces[cellAncestor[ci]],anc)<0)
              FatalErrorInFunction<<"CellSourceFaceMismatch"<<exit(FatalError);
            if(pointLevel[pi]==cellLevel[ci]-1) ++lower;
            if(pointLevel[pi]==cellLevel[ci]) ++upper;
          }
          if(lower!=3 || upper!=3) FatalErrorInFunction<<"CellLevelMismatch"<<exit(FatalError);
        }
        const fileName dumpDir(evidenceDir/("prefix_"+Foam::name(k)));
        contractDump(mesh,dumpDir);
        writeRaw(std::string(dumpDir)+"/point_ancestor.i32",pointAncestor.cdata(),pointAncestor.size());
        writeRaw(std::string(dumpDir)+"/point_level.i32",pointLevel.cdata(),pointLevel.size());
        writeRaw(std::string(dumpDir)+"/cell_ancestor.i32",cellAncestor.cdata(),cellAncestor.size());
        writeRaw(std::string(dumpDir)+"/cell_level.i32",cellLevel.cdata(),cellLevel.size());

        Info<< "Committed stage " << stagei + 1
            << ": points=" << mesh.nPoints()
            << " faces=" << mesh.nFaces()
            << " cells=" << mesh.nCells() << nl;
    }

    mesh.setInstance(runTime.constant());
    mesh.write();

    OFstream summary(evidenceDir/"summary");
    summary
        << "stages " << stageDisplacements.size() << token::END_STATEMENT << nl
        << "initialPointCount " << initialPointCount
        << token::END_STATEMENT << nl
        << "finalPointCount " << mesh.nPoints() << token::END_STATEMENT << nl
        << "finalFaceCount " << mesh.nFaces() << token::END_STATEMENT << nl
        << "finalCellCount " << mesh.nCells() << token::END_STATEMENT << nl
        << "addedCells " << labelList(allAddedCells)
        << token::END_STATEMENT << nl;

    Info<< "Repeated stock-kernel insertion complete. stages="
        << stageDisplacements.size() << " final points=" << mesh.nPoints()
        << " faces=" << mesh.nFaces() << " cells=" << mesh.nCells()
        << endl;

    return 0;
}
