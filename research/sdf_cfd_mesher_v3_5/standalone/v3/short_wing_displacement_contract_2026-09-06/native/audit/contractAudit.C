#include "argList.H"
#include "Time.H"
#include "fvMesh.H"
#include "OSspecific.H"
#include "contractDump.H"
int main(int argc,char** argv){
 argList::addOption("outputDir","path","native metric output");
 #include "setRootCase.H"
 #include "createTime.H"
 fvMesh mesh(IOobject(fvMesh::defaultRegion,runTime.constant(),runTime,IOobject::MUST_READ));
 contractDump(mesh,args.optionLookupOrDefault<fileName>("outputDir","native"));
 return 0;
}
