#pragma once

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


using namespace chrono;
using namespace rapidjson;

class LoadingSystem
{
private:

	double xPos = 0;
	double yPos = 0;
	double zPos = 0;

	ChSystemNSC system;

	double bodyDensity = 100;
	double xDim = 1;
	double yDim = 1;
	double zDim = 1;

	double maxPos = 0.0;
	double minPos = 0.0;
	double ratePos = 0.0;


	std::shared_ptr<ChBody> brick;

public:

	ChFunctionSine func;

	double functionAmpl = 0.0;
	double functionPhase = 0.0;
	double functionFreq = 0.0;

	LoadingSystem(Document&);

	void CreateBrick();
	void AddSystem(ChSystemNSC&);
	void SetVerticalPosition(double);

};

