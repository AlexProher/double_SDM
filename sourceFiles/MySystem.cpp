
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "MySystem.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


using namespace chrono;
using namespace rapidjson;

MySystem::MySystem(Document& config) {

	std::cout << "Create MySystem\n";

	xPos = config["Position"]["x"].GetDouble();
	yPos = config["Position"]["y"].GetDouble();
	zPos = config["Position"]["z"].GetDouble();
	
	if (config.HasMember("Rim")) {
		rRimDim = config["Rim"]["rRim"].GetDouble();
		std::cout << "r Rim size - " << rRimDim << "\n";

		hRimDim = config["Rim"]["hRim"].GetDouble();
		std::cout << "h Rim size - " << hRimDim << "\n";

		rimDensity = config["Rim"]["density"].GetDouble();
	}

	if (config.HasMember("Tire")) {
		rTireDim = config["Tire"]["rTire"].GetDouble();
		std::cout << "r Tire size - " << rTireDim << "\n";

		hTireDim = config["Tire"]["hTire"].GetDouble();
		std::cout << "h Tire size - " << hTireDim << "\n";

		tireDensity = config["Tire"]["density"].GetDouble();
	}

	if (config.HasMember("Floor")) {
		xFloorDim = config["Floor"]["x"].GetDouble();
		std::cout << "x Floor size - " << xFloorDim << "\n";

		zFloorDim = config["Floor"]["z"].GetDouble();
		std::cout << "h Floor size - " << xFloorDim << "\n";
	}

	if (config.HasMember("Body")) {
		xDim = config["Body"]["xSize"].GetDouble();
		std::cout << "x body size - " << xDim << "\n";

		yDim = config["Body"]["ySize"].GetDouble();
		std::cout << "y body size - " << yDim << "\n";

		zDim = config["Body"]["zSize"].GetDouble();
		std::cout << "z body size - " << zDim << "\n";

		bodyDensity = config["Body"]["density"].GetDouble();
	}

	if (config.HasMember("SD_1")) {
		bodySpring = config["SD_1"]["spring"].GetDouble();
		std::cout << "bodySpring - " << bodySpring << "\n";

		bodyDamping = config["SD_1"]["damping"].GetDouble();
		std::cout << "bodyDamping - " << bodyDamping << "\n";

		bodySuspBase = config["SD_1"]["base"].GetDouble();
		std::cout << "bodyBase - " << bodySuspBase << "\n";
	}
	if (config.HasMember("SD_2")) {
		tireSpring = config["SD_2"]["spring"].GetDouble();
		std::cout << "tireSpring - " << tireSpring << "\n";

		tireDamping = config["SD_2"]["damping"].GetDouble();
		std::cout << "tireDamping - " << tireDamping << "\n";

		tireSuspBase = config["SD_2"]["base"].GetDouble();
		std::cout << "tireBase - " << tireSuspBase << "\n";
	}

};

void MySystem::CreateFloor() {
	auto floorMat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto floorVisMat = chrono_types::make_shared<ChVisualMaterial>();
	floor = chrono_types::make_shared<ChBodyEasyBox>(xFloorDim, yFloorDim, zFloorDim, 1, true, true, floorMat);
	floor->SetPos(ChVector3d(0, -yFloorDim/2, 0));
	floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"), 100, 100);
	floor->SetFixed(true);
}

void MySystem::CreateTire() {
	auto tireMat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto tireVisMat = chrono_types::make_shared<ChVisualMaterial>();

	tire = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis(2), rTireDim, hTireDim, tireDensity, tireMat);
	tire->SetPos(ChVector3d(xPos, yPos, zPos));
	tire->EnableCollision(true);
	tire->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
}

void MySystem::CreateRim() {

	auto rimMat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto rimVisMat = chrono_types::make_shared<ChVisualMaterial>();

	rim = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis(2), rRimDim, hRimDim, rimDensity, rimMat);
	rim->SetPos(ChVector3d(xPos, yPos + tireSuspBase, zPos));
	rim->EnableCollision(false);
	rim->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));
}

void MySystem::CreateAxe() {

	rimAxis = chrono_types::make_shared<ChBody>();
	rimAxis->SetPos(ChVector3d(xPos, yPos + tireSuspBase, zPos));

	tireAxis = chrono_types::make_shared<ChBody>();
	tireAxis->SetPos(ChVector3d(xPos, yPos, zPos));
}

void MySystem::CreateBody() {
	auto bodyMat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto bodyVisMat = chrono_types::make_shared<ChVisualMaterial>();
	body = chrono_types::make_shared<ChBodyEasyBox>(xDim, yDim, zDim, bodyDensity, true, true, bodyMat);
	body->SetPos(ChVector3d(xPos, yPos + bodySuspBase + tireSuspBase, zPos));
	body->GetVisualShape(0)->SetColor(ChColor(0.8, 0.7, 0.7));

}

void MySystem::LinkBodies() {

	tireAxisLink = chrono_types::make_shared<ChLinkMateSpherical>();
	tireAxisLink->Initialize(tire, tireAxis, false, tire->GetPos(),tireAxis->GetPos());
	tireAxisLink->SetConstrainedCoords(true, true, true, true, true, false);

	rimAxisLink = chrono_types::make_shared<ChLinkMateSpherical>();
	rimAxisLink->Initialize(rim, rimAxis, false, rim->GetPos(), rimAxis->GetPos());
	rimAxisLink->SetConstrainedCoords(true, true, true, true, true, false);

	rimTireAxisLink = chrono_types::make_shared<ChLinkMateSpherical>();
	rimTireAxisLink->Initialize(rim, tireAxis, false, rim->GetPos(), tireAxis->GetPos());
	rimTireAxisLink->SetConstrainedCoords(true, false, true, true, true, true);

	bodyRimAxisLink = chrono_types::make_shared<ChLinkMateSpherical>();
	bodyRimAxisLink->Initialize(body, rimAxis, false, body->GetPos(), rimAxis->GetPos());
	bodyRimAxisLink->SetConstrainedCoords(true, false, true, true, true, true);

	holdBodyRotationLink = chrono_types::make_shared<ChLinkMateSpherical>();
	holdBodyRotationLink->Initialize(body, floor, false, body->GetPos(), floor->GetPos());
	holdBodyRotationLink->SetConstrainedCoords(false, false, false, true, true, true);

	holdRimRotationLink = chrono_types::make_shared<ChLinkMateSpherical>();
	holdRimRotationLink->Initialize(rim, floor, false, rim->GetPos(), floor->GetPos());
	holdRimRotationLink->SetConstrainedCoords(false, false, false, true, true, true);

}

void MySystem::LinkSuspention() {
	bodySuspentionLink = chrono_types::make_shared<ChLinkTSDA>();
	bodySuspentionLink->Initialize(body, rimAxis, false, body->GetPos(), rimAxis->GetPos());
	bodySuspentionLink->SetSpringCoefficient(bodySpring);
	bodySuspentionLink->SetRestLength(bodySuspBase);
	bodySuspentionLink->SetDampingCoefficient(bodyDamping);
	bodySuspentionLink->SetActuatorForce(actForce * body->GetMass());
}

void MySystem::LinkWheelSuspention() {
	tireSuspentionLink = chrono_types::make_shared<ChLinkTSDA>();
	tireSuspentionLink->Initialize(rim, tireAxis, false, rim->GetPos(), tireAxis->GetPos());
	tireSuspentionLink->SetSpringCoefficient(tireSpring);
	tireSuspentionLink->SetRestLength(tireSuspBase);
	tireSuspentionLink->SetDampingCoefficient(tireDamping);
	tireSuspentionLink->SetActuatorForce(actForce * (rim->GetMass()+ body->GetMass()));
}

ChVector3d MySystem::GetBodyPos() {
	return body->GetPos();
}

ChVector3d MySystem::GetBodyPosRel() {
	return bodySuspentionLink->GetPoint2Rel();
}

ChVector3d MySystem::GetRimPos() {
	return rim->GetPos();
}

ChVector3d MySystem::GetTirePos() {
	return tire->GetPos();
}

ChVector3d MySystem::GetRimPosRel() {
	return tireSuspentionLink->GetPoint2Rel();
}

void MySystem::SetRimPos(ChVector3d pos) {
	return rim->SetPos(pos);
}

void MySystem::SetTireVel(double xVel) {
	tire->SetLinVel(ChVector3d(xVel, 0, 0));
}

void MySystem::UpdateActForce(double controlForce) {
	bodySuspentionLink->SetActuatorForce(actForce * body->GetMass() + controlForce);
}


void MySystem::AddSystem(ChSystemNSC& sys) {
	CreateFloor();
	sys.AddBody(floor);

	CreateRim();
	sys.AddBody(rim);

	CreateTire();
	sys.AddBody(tire);

	CreateBody();
	CreateAxe();
	sys.AddBody(body);
	sys.AddBody(tireAxis);
	sys.AddBody(rimAxis);

	LinkBodies();

	LinkSuspention();
	LinkWheelSuspention();

	sys.AddLink(bodySuspentionLink);
	sys.AddLink(tireSuspentionLink);

	sys.AddLink(tireAxisLink);
	sys.AddLink(rimAxisLink);

	sys.AddLink(rimTireAxisLink);
	sys.AddLink(bodyRimAxisLink);

	sys.AddLink(holdBodyRotationLink);
	sys.AddLink(holdRimRotationLink);


}