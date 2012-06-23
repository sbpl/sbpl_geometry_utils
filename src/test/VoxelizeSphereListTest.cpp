#include <iostream>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <vector>

using namespace std;

int main(int argc, char** argv)
{
    cout << "Hello, VoxelizeSphereListTest" << endl;

    vector<vector<double> > spheres; spheres.clear();

    vector<double> sphere;
    sphere.push_back(0.0);
    sphere.push_back(0.0);
    sphere.push_back(0.0);
    sphere.push_back(0.98);

    vector<double> sphere2;
    sphere2.push_back(0.5);
    sphere2.push_back(0.0);
    sphere2.push_back(0.0);
    sphere2.push_back(0.98);

    spheres.push_back(sphere);
    spheres.push_back(sphere2);

    vector<std::vector<double> > voxels; voxels.clear();
    double volumeOut = 0.0;

    sbpl::Voxelizer::voxelizeSphereList(spheres, 0.5, true, voxels, volumeOut);

    for (int i = 0; i < (int)voxels.size(); i++) {
        std::vector<double>& voxel = voxels[i];
        cout << "{" << voxel[0] << ", " << voxel[1] << ", " << voxel[2] << "}" << endl;
    }

    cout << "The voxels take up " << volumeOut << " units cubed." << endl;

    return 0;
}
