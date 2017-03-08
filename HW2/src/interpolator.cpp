#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <fstream>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
	//Set default interpolation type
	m_InterpolationType = LINEAR;

	//set default angle representation to use for interpolation
	m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N)
{
	//Allocate new motion
	*pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton());

	//Perform the interpolation
	if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
		LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
		LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
		BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
		BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else
	{
		printf("Error: unknown interpolation / angle representation type.\n");
		exit(1);
	}
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
				interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
	double cy = sqrt(R[0] * R[0] + R[3] * R[3]);

	if (cy > 16 * DBL_EPSILON)
	{
		angles[0] = atan2(R[7], R[8]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = atan2(R[3], R[0]);
	}
	else
	{
		angles[0] = atan2(-R[5], R[4]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = 0;
	}

	for (int i = 0; i < 3; i++)
		angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
	// students should implement this
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	// students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
	if (2 * N + 2 >= inputLength)
	{
		printf("Too Few Control Frames\n");
		return;
	}
		
	int startKeyframe = 0;
	
	while (startKeyframe + N + 1 < inputLength)
	{		
		int endKeyframe = startKeyframe + N + 1;

		Posture *startPosture, *endPosture;
		
		startPosture = pInputMotion->GetPosture(startKeyframe);
		endPosture = pInputMotion->GetPosture(endKeyframe);
		
		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{			
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position			
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				vector pPre, pStart, pEnd, pNext;
				std::vector<vector> controlPoints;
				if (!(startPosture->bone_rotation[bone] == endPosture->bone_rotation[bone]))
				{
					pStart = startPosture->bone_rotation[bone].p;
					pEnd = endPosture->bone_rotation[bone].p;
					int controlVal = 0;
					if (startKeyframe == 0)
						controlVal = 1;
					else
					{
						Posture *prePosture = pInputMotion->GetPosture(startKeyframe - N - 1);
						pPre = prePosture->bone_rotation[bone].p;
					}
					if (endKeyframe + N + 1 < inputLength)
					{
						Posture *nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
						pNext = nextPosture->bone_rotation[bone].p;
					}
					else
						controlVal = 2;
					controlPoints = GenerateControlPoints(controlVal, pPre, pStart, pEnd, pNext);
					interpolatedPosture.bone_rotation[bone] = 
						DeCasteljauEuler(t, startPosture->bone_rotation[bone], controlPoints[0], controlPoints[1], endPosture->bone_rotation[bone]);
				}
				else
					interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone];
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	// students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Quaternion<double> start = Quaternion<double>(), end = Quaternion<double>();
				if (!(startPosture->bone_rotation[bone] == endPosture->bone_rotation[bone]))
				{
					Euler2Quaternion(startPosture->bone_rotation[bone].p, start);
					Euler2Quaternion(endPosture->bone_rotation[bone].p, end);
					start.Normalize();
					end.Normalize();
					double interpolateAngle[3];
					Quaternion2Euler(Slerp(t, start, end), interpolateAngle);
					interpolatedPosture.bone_rotation[bone] = vector(interpolateAngle);
				}
				else
					interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone];
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	// students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
	if (2 * N + 2 >= inputLength)
	{
		printf("Too Few Control Frames\n");
		return;
	}

	int startKeyframe = 0;	
	while (startKeyframe + N + 1 < inputLength)
	{		
		int endKeyframe = startKeyframe + N + 1;

		Posture *startPosture, *endPosture;

		startPosture = pInputMotion->GetPosture(startKeyframe);
		endPosture = pInputMotion->GetPosture(endKeyframe);
		
		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position			
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{				
				Quaternion<double> qpresbone = Quaternion<double>(), qsbone = Quaternion<double>(),
					qebone = Quaternion<double>(), qnexbone = Quaternion<double>();
				std::vector<Quaternion<double>> bonecontrolPoints;
				Quaternion<double> decaste_result;
				if (!(startPosture->bone_rotation[bone] == endPosture->bone_rotation[bone]))
				{
					Euler2Quaternion(startPosture->bone_rotation[bone].p, qsbone);
					Euler2Quaternion(endPosture->bone_rotation[bone].p, qebone);
					int controlVal = 0;
					if (startKeyframe != 0 && startKeyframe >= N - 1)
					{
						Posture *prePosture = pInputMotion->GetPosture(startKeyframe - N - 1);
						Euler2Quaternion(prePosture->bone_rotation[bone].p, qpresbone);
					}
					else
						controlVal = 1;
					if (endKeyframe + N + 1 < inputLength)
					{
						Posture *nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
						Euler2Quaternion(nextPosture->bone_rotation[bone].p, qnexbone);
					}
					else
						controlVal = 2;
					qpresbone.Normalize();
					qsbone.Normalize();
					qebone.Normalize();
					qnexbone.Normalize();
					bonecontrolPoints = GenerateControlPoints(controlVal, qpresbone, qsbone, qebone, qnexbone);
					decaste_result = DeCasteljauQuaternion(t, qsbone, bonecontrolPoints[0], bonecontrolPoints[1], qebone);
					Quaternion2Euler(decaste_result, interpolatedPosture.bone_rotation[bone].p);
				}
				else
					interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone];
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
	// students should implement this
	// angles contains Euler angles of Roll(x), Pitch(y) and Yaw(z) in degrees
	double thetax = angles[0], thetay = angles[1], thetaz = angles[2];
	thetax = thetax*M_PI / 180;
	thetay = thetay*M_PI / 180;
	thetaz = thetaz*M_PI / 180;
	double w = cos(thetax / 2)*cos(thetay / 2)*cos(thetaz / 2) + sin(thetax / 2)*sin(thetay / 2)*sin(thetaz / 2);
	double x = sin(thetax / 2)*cos(thetay / 2)*cos(thetaz / 2) - cos(thetax / 2)*sin(thetay / 2)*sin(thetaz / 2);
	double y = cos(thetax / 2)*sin(thetay / 2)*cos(thetaz / 2) + sin(thetax / 2)*cos(thetay / 2)*sin(thetaz / 2);
	double z = cos(thetax / 2)*cos(thetay / 2)*sin(thetaz / 2) - sin(thetax / 2)*sin(thetay / 2)*cos(thetaz / 2);
	q.Set(w, x, y, z);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3])
{
	// students should implement this
	// angles contains Euler angles of Roll(x), Pitch(y) and Yaw(z) in degrees
	double s = q.Gets(), x = q.Getx(), y = q.Gety(), z = q.Getz();
	angles[0] = atan2(2 * (s*x + y*z), 1 - 2 * (x*x + y*y)) * 180 / M_PI;
	angles[1] = asin(2 * (s*y - z*x)) * 180 / M_PI;
	angles[2] = atan2(2 * (s*z + x*y), 1 - 2 * (y*y + z*z)) * 180 / M_PI;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
	// students should implement this
	Quaternion<double> result;
	double tmp = qStart.Gets()*qEnd_.Gets() + qStart.Getx()*qEnd_.Getx()
		+ qStart.Gety()*qEnd_.Gety() + qStart.Getz()*qEnd_.Getz();
	if (tmp > 1)
		tmp = 1;
	else if (tmp < -1)
		tmp = -1;
	double theta = acos(tmp);
	if (abs(theta) < EQ_LIMIT)
		return qStart;
	result = qStart * sin((1 - t)*theta) / sin(theta) + qEnd_ * sin(t*theta) / sin(theta);
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
	// students should implement this
	Quaternion<double> result;
	result = 2 * p.innerproduct(q) * q - p;
	return result;
}

vector Interpolator::Linear(double t, vector pStart, vector pEnd)
{
	vector res = pStart * (1-t) + pEnd * t;
	return res;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
	// students should implement this
	vector result = vector();
	result = p0 * pow((1 - t), 3) + p1 * 3 * t*pow((1 - t), 2) + p2 * 3 * t*t*(1 - t) + p3 * pow(t, 3);
	return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
	// students should implement this
	// p0 and p3 is the start and end points
	// p1 and p2 are 2 control points
	Quaternion<double> result;
	Quaternion<double> Q0 = Slerp(t, p0, p1);
	Quaternion<double> Q1 = Slerp(t, p1, p2);
	Quaternion<double> Q2 = Slerp(t, p2, p3);
	Q0 = Slerp(t, Q0, Q1);
	Q1 = Slerp(t, Q1, Q2);
	result = Slerp(t, Q0, Q1);
	return result;
}

std::vector<Quaternion<double>> Interpolator::GenerateControlPoints(int boundcondition, Quaternion<double> q0, Quaternion<double> q1, Quaternion<double> q2, Quaternion<double> q3)
{
	// q0 is the previous quaternion of q1
	// q1 is the start quaternion
	// q2 is the end quaternion
	// q3 is the next quaternion of q2
	// if q1 comes from the first control frame, boundcondition == 1
	// if q2 comes from the last control frame, boundcondition == 2
	Quaternion<double> controlA, controlB;
	if (boundcondition == 2)
	{
		controlA = Slerp(1.0 / 3, q1, Slerp(0.5, Slerp(2.0, q0, q1), q2));
		controlB = Slerp(1.0 / 3, q2, Slerp(2.0, q0, q1));
	}
	else if (boundcondition == 1)
	{

		controlA = Slerp(1.0 / 3, q1, Slerp(2.0, q3, q2));
		controlB = Slerp(-1.0 / 3, q2, Slerp(0.5, Slerp(2.0, q1, q2), q3));
	}
	else
	{
		controlA = Slerp(1.0 / 3, q1, Slerp(0.5, Slerp(2.0, q0, q1), q2));
		controlB = Slerp(-1.0 / 3, q2, Slerp(0.5, Slerp(2.0, q1, q2), q3));
	}
	std::vector<Quaternion<double>> res;
	res.push_back(controlA);
	res.push_back(controlB);
	return res;
}

std::vector<vector> Interpolator::GenerateControlPoints(int boundcondition, vector p0, vector p1, vector p2, vector p3)
{
	// p0 is the previous Euler Angle of p1
	// p1 is the start Euler Angle
	// p2 is the end Euler Angle
	// p3 is the next Euler Angle of p2
	// if p1 comes from the first control frame, boundcondition == 1
	// if p2 comes from the last control frame, boundcondition == 2
	vector controlA, controlB;
	if (boundcondition == 2)
	{
		controlA = Linear(1.0 / 3, p1, Linear(0.5, Linear(2.0, p0, p1), p2));
		controlB = Linear(1.0 / 3, p2, Linear(2.0, p0, p1));
	}
	else if (boundcondition == 1)
	{

		controlA = Linear(1.0 / 3, p1, Linear(2.0, p3, p2));
		controlB = Linear(-1.0 / 3, p2, Linear(0.5, Linear(2.0, p1, p2), p3));
	}
	else
	{
		controlA = Linear(1.0 / 3, p1, Linear(0.5, Linear(2.0, p0, p1), p2));
		controlB = Linear(-1.0 / 3, p2, Linear(0.5, Linear(2.0, p1, p2), p3));
	}
	std::vector<vector> res;
	res.push_back(controlA);
	res.push_back(controlB);
	return res;
}
