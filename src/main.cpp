#include <cstdlib>
#include <string>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "Settings.h"

using namespace nl_uu_science_gmt;

int main(
		int argc, char** argv)
{
	//getAveragePicture("data/cam1/background.avi");
	//waitKey(0);
	//calibrateAndSave();
	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}
