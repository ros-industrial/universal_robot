
#include "ur_ctrl_server/ur_hardware_controller.h"
#include "run_controller.cpp"

int main(int argc, char** argv) { return runController<ur::URController>(argc, argv); }
