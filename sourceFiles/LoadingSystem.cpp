#include "LoadingSystem.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/functions/ChFunctionSine.h"


using namespace chrono;
using namespace rapidjson;


LoadingSystem::LoadingSystem(Document& config) {
	std::cout << "Create LoadingSystem\n";

	xPos = config["Position"]["x"].GetDouble();
	yPos = config["Position"]["y"].GetDouble();
	zPos = config["Position"]["z"].GetDouble();

	xDim = config["Dimention"]["x"].GetDouble();
	yDim = config["Dimention"]["y"].GetDouble();
	zDim = config["Dimention"]["z"].GetDouble();

	functionAmpl = config["VerticalDisplacement"]["amplitude"].GetDouble();;
	functionPhase = config["VerticalDisplacement"]["phase"].GetDouble();;
	functionFreq = config["VerticalDisplacement"]["freq"].GetDouble();;

	maxPos = config["VerticalDisplacement"]["maxPos"].GetDouble();
	minPos = config["VerticalDisplacement"]["minPos"].GetDouble();
	ratePos = config["VerticalDisplacement"]["rate"].GetDouble();

	func = ChFunctionSine(
		functionAmpl, 
		functionFreq, 
		functionPhase
	);

};

void LoadingSystem::CreateBrick() {
	auto brickMat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto brickVisMat = chrono_types::make_shared<ChVisualMaterial>();
	brick = chrono_types::make_shared<ChBodyEasyBox>(xDim, yDim, zDim, 1, true, true, brickMat);
	brick->SetPos(ChVector3d(xPos, yPos - yDim / 2, zPos));
	brick->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"), 50, 50);
	brick->SetFixed(true);
};

void LoadingSystem::AddSystem(ChSystemNSC& sys) {
	CreateBrick();
	sys.AddBody(brick);
};

void LoadingSystem::SetVerticalPosition(double dt) {

	brick->SetPos(ChVector3d(xPos, (yPos - yDim / 2) - func.GetVal(dt), zPos));
	brick->SetRot(ChQuaternion(0, 0, 0, 0));
	
};
