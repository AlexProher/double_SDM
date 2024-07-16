#pragma once

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


using namespace chrono;
using namespace rapidjson;

class MySystem {

private:

	double xPos;
	double yPos;
	double zPos;

	double actForce = 9.8f;

	ChSystemNSC system;

	double tireDamping;
	double bodyDamping;
	double tireSpring;
	double bodySpring;

	std::shared_ptr<ChLinkTSDA> bodySuspentionLink;
	std::shared_ptr<ChLinkTSDA> tireSuspentionLink;

	std::shared_ptr<ChLinkMateSpherical> tireAxisLink;
	std::shared_ptr<ChLinkMateSpherical> rimAxisLink;

	std::shared_ptr<ChLinkMateSpherical> rimTireAxisLink;
	std::shared_ptr<ChLinkMateSpherical> bodyRimAxisLink;

	std::shared_ptr<ChLinkMateSpherical> holdBodyRotationLink;
	std::shared_ptr<ChLinkMateSpherical> holdRimRotationLink;

	std::shared_ptr<ChForce> frc = chrono_types::make_shared<ChForce>();


	double bodyDensity = 100;
	double xDim = 1;
	double yDim = 1;
	double zDim = 1;

	double rimDensity = 1000;
	double hRimDim = 1;
	double rRimDim = 1;

	double tireDensity = 1000;
	double hTireDim = 1;
	double rTireDim = 1;

	double xFloorDim = 10;
	double yFloorDim = 0.1f;
	double zFloorDim = 10;

	double bodySuspBase = 3;
	double tireSuspBase = 0;

	std::shared_ptr<ChBody> body;
	std::shared_ptr<ChBody> rim;
	std::shared_ptr<ChBody> tire;
	std::shared_ptr<ChBody> rimAxis;
	std::shared_ptr<ChBody> tireAxis;
	std::shared_ptr<ChBody> floor;

public:

	MySystem(Document&);
	void AddSystem(ChSystemNSC&);

	void CreateFloor();

	void CreateTire();

	void CreateRim();

	void CreateAxe();

	void CreateBody();

	ChVector3d GetBodyPos();

	ChVector3d GetBodyPosRel();

	ChVector3d GetRimPos();

	ChVector3d GetRimPosRel();

	void SetRimPos(ChVector3d);

	void SetTireVel(double xVel);

	void UpdateActForce(double);

	void LinkBodies();

	void LinkSuspention();

	void LinkWheelSuspention();
	
};