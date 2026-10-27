#include "polyMeshCheck.H"
#include "primitiveMeshCheck.H"
#include "syncTools.H"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <vector>
using namespace Foam;
template<class T>
static void writeRaw(const std::string& path, const T* data, const std::size_t count)
{
    std::ofstream os(path, std::ios::binary);
    os.write(reinterpret_cast<const char*>(data), count*sizeof(T));
}

static void contractDump(const fvMesh& mesh,const fileName& outputDir)
{
    mkDir(outputDir);
    const std::string base(outputDir.c_str());
    const pointField& points = mesh.points();
    const faceList& faces = mesh.faces();
    const labelList& own = mesh.faceOwner();
    const labelList& nei = mesh.faceNeighbour();
    const labelListList& cellPoints = mesh.cellPoints();
    const vectorField& faceAreas = mesh.faceAreas();
    const vectorField& faceCentres = mesh.faceCentres();
    const vectorField& cellCentres = mesh.cellCentres();
    const scalarField& cellVolumes = mesh.cellVolumes();

    tmp<scalarField> tDeterminant = meshCheck::cellDeterminant
    (
        mesh,
        mesh.geometricD(),
        faceAreas,
        syncTools::getInternalOrCoupledFaces(mesh)
    );
    tmp<scalarField> tWeights = meshCheck::faceWeights
    (
        mesh,
        faceCentres,
        faceAreas,
        cellCentres
    );
    tmp<scalarField> tRatios = meshCheck::volRatio(mesh, cellVolumes);
    scalarField openness, aspectRatio;
    meshCheck::cellClosedness
    (
        mesh,
        mesh.geometricD(),
        faceAreas,
        cellVolumes,
        openness,
        aspectRatio
    );
    tmp<scalarField> tSkewness = meshCheck::faceSkewness
    (
        mesh,
        points,
        faceCentres,
        faceAreas,
        cellCentres
    );
    const scalarField& determinant = tDeterminant();
    const scalarField& weights = tWeights();
    const scalarField& ratios = tRatios();
    const scalarField& skewness = tSkewness();

    std::vector<double> pointValues(3*points.size());
    forAll(points, i)
    {
        pointValues[3*i] = points[i].x();
        pointValues[3*i + 1] = points[i].y();
        pointValues[3*i + 2] = points[i].z();
    }
    writeRaw(base + "/points.f64", pointValues.data(), pointValues.size());

    std::vector<std::uint64_t> faceOffsets(faces.size() + 1, 0);
    std::vector<std::int32_t> facePoints;
    facePoints.reserve(4*faces.size());
    forAll(faces, facei)
    {
        faceOffsets[facei] = facePoints.size();
        forAll(faces[facei], fp)
        {
            facePoints.push_back(static_cast<std::int32_t>(faces[facei][fp]));
        }
    }
    faceOffsets[faces.size()] = facePoints.size();
    writeRaw(base + "/face_offsets.u64", faceOffsets.data(), faceOffsets.size());
    writeRaw(base + "/face_points.i32", facePoints.data(), facePoints.size());

    std::vector<std::int32_t> owner(own.size()), neighbour(nei.size());
    forAll(own, i) owner[i] = static_cast<std::int32_t>(own[i]);
    forAll(nei, i) neighbour[i] = static_cast<std::int32_t>(nei[i]);
    writeRaw(base + "/owner.i32", owner.data(), owner.size());
    writeRaw(base + "/neighbour.i32", neighbour.data(), neighbour.size());

    std::vector<std::uint64_t> cellOffsets(cellPoints.size() + 1, 0);
    std::vector<std::int32_t> cellPointValues;
    cellPointValues.reserve(6*cellPoints.size());
    forAll(cellPoints, celli)
    {
        cellOffsets[celli] = cellPointValues.size();
        std::vector<label> row(cellPoints[celli].begin(), cellPoints[celli].end());
        std::sort(row.begin(), row.end());
        for (const label pointi : row)
        {
            cellPointValues.push_back(static_cast<std::int32_t>(pointi));
        }
    }
    cellOffsets[cellPoints.size()] = cellPointValues.size();
    writeRaw(base + "/cell_offsets.u64", cellOffsets.data(), cellOffsets.size());
    writeRaw(base + "/cell_points.i32", cellPointValues.data(), cellPointValues.size());

    std::vector<double> centreValues(3*cellCentres.size());
    forAll(cellCentres, i)
    {
        centreValues[3*i] = cellCentres[i].x();
        centreValues[3*i + 1] = cellCentres[i].y();
        centreValues[3*i + 2] = cellCentres[i].z();
    }
    writeRaw(base + "/cell_centres.f64", centreValues.data(), centreValues.size());
    writeRaw(base + "/cell_volumes.f64", cellVolumes.cdata(), cellVolumes.size());
    writeRaw(base + "/cell_determinants.f64", determinant.cdata(), determinant.size());
    writeRaw(base + "/face_weights.f64", weights.cdata(), nei.size());
    writeRaw(base + "/face_volume_ratios.f64", ratios.cdata(), nei.size());
    writeRaw(base + "/cell_aspect_ratios.f64", aspectRatio.cdata(), aspectRatio.size());
    writeRaw(base + "/cell_openness.f64", openness.cdata(), openness.size());
    writeRaw(base + "/face_skewness.f64", skewness.cdata(), skewness.size());


    std::vector<double> fc(3*faceCentres.size()), fa(3*faceAreas.size());
    forAll(faceCentres,i) for(direction j=0;j<3;++j) {fc[3*i+j]=faceCentres[i][j];fa[3*i+j]=faceAreas[i][j];}
    writeRaw(base+"/face_centres.f64",fc.data(),fc.size());
    writeRaw(base+"/face_areas.f64",fa.data(),fa.size());
    const PackedBoolList masks(syncTools::getInternalOrCoupledFaces(mesh));
    std::vector<std::uint8_t> im(masks.size());
    forAll(masks,i) im[i]=masks[i]?1:0;
    writeRaw(base+"/internal_mask.u8",im.data(),im.size());
}
