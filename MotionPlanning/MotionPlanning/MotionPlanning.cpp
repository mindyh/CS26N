/*
 * File: MotionPlanning.cpp
 * ----------------------------
 * Name: Mindy Huang
 * CS26N
 * store paths between pegs
 */

#include <iostream>
#include <cmath>
#include "queue.h"
#include "strlib.h"
#include <string>
#include "gevents.h"
#include "grid.h"
#include "graphics.h"
#include "gtypes.h"
#include "simpio.h"
#include "set.h"
#include "random.h"
using namespace std;

/* Constants */
#define PI 3.14159265
#define PAUSE_TIME 25

const double PEG_DIAM = 10;
const double OBST_DIAM = 40;
const double ROBOT_LENGTH = 30;
const double ROBOT_WIDTH = 10;
const double INIT_THETA = 45;

struct Orientation{
	GPoint anchor;
	GPoint joint;
	GPoint endpt;
	double theta1;
	double theta2;
};

struct Node{
	GPoint point;
	bool flipped;
};

/* Function prototypes */

void initialization(Vector<GPoint> & pegs, Vector<GPoint> & obst, GPoint & start, GPoint & goal, GMouseEvent & e);
void inputPegs(Vector<GPoint> & pegs, GMouseEvent & e);
void inputObstacles(Vector<GPoint> & obst, GMouseEvent & e);
void inputStart(GPoint & start, GMouseEvent & e);
void inputGoal(GPoint & goal, GMouseEvent & e);
void message(string str);
GPoint getClick(GMouseEvent & e);
string vecToString(Vector<GPoint> & pts);
void drawRobot(Orientation & o, string color);
void drawRobot(GPoint & anchor, double & theta1, double & theta2, string color);
Orientation inverseKinematics(GPoint & nextPt, GPoint & anchor, bool flag);
double dotProduct(GPoint & p1, GPoint & p2);
double acosd(double x);
double cosd(double x);
double sind(double x);
double distance (double x1, double y1, double x2, double y2);
Vector<Orientation> PRM(Node & nextNode, Orientation & robot, Vector<GPoint> & obst);
//Vector<Orientation> choosePath(Vector<GPoint> & samplePoints, Orientation & robot, GPoint & goal, Vector<GPoint> & obst);
//GPoint getClosestPt(Vector<GPoint> & samplePoints, GPoint & lastPoint);
Vector<GPoint> getPegsInRange(Vector<GPoint> & pegs, GPoint & anchor);
bool inRange(GPoint & p1, GPoint & p2);
//Vector<GPoint> pickPoints(GPoint & origin);
bool isColliding(Orientation & robot, Vector<GPoint> & obstacles);
void removePtFromVec(Vector<GPoint> & vec, GPoint & pt);
Vector<Orientation> genSmallPath(Orientation & curr, Orientation & final);
void displayPath(Vector<Orientation> & orientations, Vector<GPoint> & pegs, Vector<GPoint> & obst);
double asind(double x);
double atand(double x);
void updateOrientation(Orientation & curr, Orientation & goal);
GPoint getNextValidPt(Vector<GPoint> & pegs, Orientation & robot, Vector<Orientation> & path, Vector<GPoint> & obst);
bool isValidPegPath(Vector<Node> & path, Vector<GPoint> & obst, Orientation & robot, 
				 Vector< Vector<Orientation> > & prmPaths);
void drawObst(GPoint & p);
void redrawObst(Vector<GPoint> & obst);
void drawPeg(GPoint & p, string color);
void redrawPegs(Vector<GPoint> & obst);
bool vecContains(Vector<GPoint> & vec, GPoint & pt);
Vector<GPoint> getValidPegs(Vector<GPoint> & pegsInRange, Vector<GPoint> & usedPegs);
Vector<Node> genPath(GPoint & start, GPoint & goal, Vector<GPoint> & pegs, 
					   Vector<GPoint> & obst, Orientation & robot, Vector< Vector<Orientation> > & prmPaths);
void incrementTheta(double & theta, double & goal);
void fixTheta(double & theta1, double & theta2);
Vector<GPoint> randomSample(GPoint & origin, Orientation & robot);
bool isValidPRMPath(Vector<GPoint> & prmPath, Vector<GPoint> & obst, Orientation & robot);
Vector<Node> genPRMPath(GPoint & start, Node & goal, Vector<Node> & nodes, 
					   Vector<GPoint> & obst, Orientation & robot);
Vector<Node> trimSample(Vector<Node> & sample, Orientation & robot, Vector<GPoint> & obst);
Orientation calculateJoints(GPoint & anchor, double theta1, double theta2);
bool failsFirstLevelCheck(Vector<Node> & path, Vector<GPoint> & obst);
Vector<Node> createNodeVec(Vector<GPoint> & pegs);
bool nodeVecContains(Vector<Node> & vec, Node & nd);
string nodeVecToString(Vector<Node> & nodes);
void removeNodeFromVec(Vector<Node> & vec, Node & n);

/* Main program */

int main() {
	initGraphics();
	Vector<GPoint> pegs;
	Vector<GPoint> obst;
	GPoint start;
	GPoint goal;
	GMouseEvent e;
	initialization(pegs, obst, start, goal, e); 

	Orientation robot;
	robot.anchor = start;
	robot.theta1 = INIT_THETA;
	robot.theta2 = 360-INIT_THETA;
	drawRobot(start, robot.theta1, robot.theta2, "BLACK");

	Vector< Vector<Orientation> > prmPaths;
	Vector<Orientation> smallPath;
	Vector<Node> pegPath = genPath(start, goal, pegs, obst, robot, prmPaths);
	for(int i = 1; i < pegPath.size(); i++){
		for(int j = 0; j < (prmPaths[i].size()-1); j++){
			smallPath = genSmallPath(prmPaths[i][j], prmPaths[i][j+1]);
			displayPath(smallPath, pegs, obst);
		}
	}

	cout <<"done"<< endl;

	return 0;
} 

Vector<GPoint> getValidPegs(Vector<GPoint> & pegsInRange, Vector<GPoint> & usedPegs){
	Vector<GPoint> retval;
	foreach(GPoint p in pegsInRange){
		if(!vecContains(usedPegs, p)) 
			retval.add(p);
	}
	return retval;
}

Vector<Node> getValidNodes(Vector<GPoint> & pegsInRange, Vector<Node> & usedNodes){
	Vector<Node> retval;
	Vector<Node> nodesInRange = createNodeVec(pegsInRange);
	foreach(Node n in nodesInRange){
		if(!nodeVecContains(usedNodes, n)) retval.add(n);
	}
	return retval;
}

Vector<Node> createNodeVec(Vector<GPoint> & pegs){
	Vector<Node> retval;
	foreach(GPoint p in pegs){
		Node n1;
		n1.point = p;
		n1.flipped = true;
		Node n2;
		n2.point = p;
		n2.flipped = false;
		retval.add(n1);
		retval.add(n2);
	}
	return retval;
}

Vector<Node> genPath(GPoint & start, GPoint & goal, Vector<GPoint> & pegs, 
					   Vector<GPoint> & obst, Orientation & robot, Vector< Vector<Orientation> > & prmPaths){
	Vector<Node> nodes = createNodeVec(pegs);
	Queue< Vector<Node> > fringe;
	Vector<Node> vec;
	Map< string, Vector<Node> > map;
	Vector<Node> usedNodes;

	Node startNode;
	startNode.point = start;
	startNode.flipped = true;
	vec.add(startNode);

	map.put(nodeVecToString(vec), usedNodes);
	fringe.enqueue(vec);
	GPoint currPeg;

	while(!fringe.isEmpty()){
		vec = fringe.dequeue();
		if(vec[vec.size() - 1].point == goal) {
			if(isValidPegPath(vec, obst, robot, prmPaths)) return vec;
			else {
				cout << "fringe size: " << fringe.size() << endl;
				continue;
				cout << "not supposed to be in here: genPath" << endl;
			}
		}
		usedNodes = map[nodeVecToString(vec)];
		currPeg = vec[vec.size() - 1].point;
		Vector<Node> validNodes = getValidNodes(getPegsInRange(pegs, currPeg), usedNodes);
		for (int i = 0; i < validNodes.size(); i++){
			if(nodeVecContains(usedNodes, validNodes[i])) continue;
			Vector<Node> usedNodes2 = usedNodes;
			Vector<Node> vec2 = vec;
			vec2.add(validNodes[i]);
			fringe.enqueue(vec2);
			usedNodes2.add(validNodes[i]);

			map.put(nodeVecToString(vec2), usedNodes2);
		}
	}
	cout << "no path found" << endl;
	vec.clear();
	return vec;
}

bool vecContains(Vector<GPoint> & vec, GPoint & pt){
	foreach(GPoint p in vec){
		if (p == pt) return true;
	}
	return false;
}

bool nodeVecContains(Vector<Node> & vec, Node & nd){
	foreach(Node n in vec){
		if (n.point == nd.point &&
			n.flipped == nd.flipped) return true;
	}
	return false;
}

//GPoint getNextValidPt(Vector<GPoint> & pegs, Orientation & robot, Vector<Orientation> & path, Vector<GPoint> & obst){
//	GPoint nextPt;
//	while(true){
//		if(pegs.isEmpty()){
//			cout << "Stuck" << endl;
//			return GPoint(0,0); 
//		}
//		nextPt = getClosestPt(pegs, robot.anchor);
//		path = genSmallPath(robot, inverseKinematics(nextPt, robot)); 
//		if(isValidPath(path, obst)) return nextPt;
//		removePtFromVec(pegs, nextPt);
//	}
//}

bool failsFirstLevelCheck(Vector<Node> & path, Vector<GPoint> & obst){
	Orientation o;
	for(int i = 0; i < path.size()-1; i++){
		o = inverseKinematics(path[i].point, path[i+1].point, path[i].flipped);
		if(isColliding(o, obst)) return true;
	}
	return false;
}

bool isValidPegPath(Vector<Node> & path, Vector<GPoint> & obst, Orientation & robot, 
				 Vector< Vector<Orientation> > & prmPaths){
	Orientation currO = robot;
	Vector<Orientation> prmPath;
	if(failsFirstLevelCheck(path, obst)) return false;
	cout << "num pegs in path: " << path.size() << endl;
	for(int i = 0; i < path.size(); i++){
		prmPath = PRM(path[i], currO, obst);
		cout<< prmPath.size() << endl;

		if(prmPath.size() == 0){
			prmPaths.clear();
			return false;
		}
 
		prmPaths.add(prmPath);
		currO = prmPath[prmPath.size()-1];

		updateOrientation(currO, currO);
	}
	return true;
}

void updateOrientation(Orientation & curr, Orientation & goal){
	//curr = goal;
	double tempTheta = curr.theta1 + 180;
	curr.theta1 = curr.theta2 + 180;
	curr.theta2 = tempTheta;

	GPoint tempPt = curr.anchor;
	curr.anchor = curr.endpt;
	curr.endpt = tempPt;
}

void fixTheta(double & theta1, double & theta2){
	if (theta1 < 0) theta1 += 360;
	if (theta2 < 0) theta2 += 360;

	if (theta1 >= 360) theta1 -= 360;
	if (theta2 >= 360) theta2 -= 360;
}

void incrementTheta(double & theta, double & goal){
	if(theta < goal && abs(theta - goal) < 180) theta++;
	else if(theta < goal && abs(theta - goal) >= 180) theta--;
	else if(theta > goal && abs(theta - goal) < 180) theta--;
	else if(theta > goal && abs(theta - goal) >= 180) theta++;
	else cout << "something's wrong: incrementTheta" << endl;
}

Vector<Orientation> genSmallPath(Orientation & curr, Orientation & final){
	Vector<Orientation> retval;
	double counterTheta1 = curr.theta1;
	double counterTheta2 = curr.theta2;
	
	fixTheta(counterTheta1, counterTheta2);
	fixTheta(final.theta1, final.theta2);

	while(abs(counterTheta1 - final.theta1) > 2){
		incrementTheta(counterTheta1, final.theta1);
		if(abs(counterTheta2 - final.theta2) > 2) incrementTheta(counterTheta2, final.theta2);

		fixTheta(counterTheta1, counterTheta2);
		Orientation temp = calculateJoints(curr.anchor, counterTheta1, counterTheta2);
		retval.add(temp);
	}

	while(abs(counterTheta2 - final.theta2) > 2){
		incrementTheta(counterTheta2, final.theta2);
		fixTheta(counterTheta1, counterTheta2);
		Orientation temp = calculateJoints(curr.anchor, counterTheta1, counterTheta2);
		retval.add(temp);
	}

	retval.add(final);
	return retval;
}

Orientation calculateJoints(GPoint & anchor, double theta1, double theta2){
	Orientation retval;
	retval.anchor = anchor;
	retval.joint = GPoint(anchor.getX() + ROBOT_LENGTH*cosd(theta1),
					anchor.getY() - ROBOT_LENGTH*sind(theta1));
	//cout << sin(o.theta1) << endl;
	retval.endpt = GPoint(retval.joint.getX() + ROBOT_LENGTH*cosd(theta2),
				retval.joint.getY() - ROBOT_LENGTH*sind(theta2));
	return retval;
}

		

//void inverseKinematics(GPoint & click, Orientation & robot){
//	double d = sqrt(pow(click.getX() - robot.anchor.getX(),2) + pow(click.getY()-robot.anchor.getY(),2));
//	if(d > 2*ROBOT_LENGTH) {
//		cout << "not in range" << endl;
//		return;
//	}
//
//	double x = (click.getX()+robot.anchor.getX())/2 + 
//		((click.getY() - robot.anchor.getY())/(2*d*d))*
//		sqrt((pow(2*ROBOT_LENGTH,2) - d*d)*d*d);
//
//	double y = (click.getY()+robot.anchor.getY())/2 - 
//		((click.getX() - robot.anchor.getX())/(2*d*d))*
//		sqrt((pow(2*ROBOT_LENGTH,2) - d*d)*d*d);
//
//	GPoint p(x,y);
//
//	drawLine(robot.anchor, p);
//	drawLine(p, click);
//}

void displayPath(Vector<Orientation> & orientations, Vector<GPoint> & pegs, Vector<GPoint> & obst){
	drawRobot(orientations[0], "BLACK");
	for(int i = 1; i < orientations.size(); i++){
		drawRobot(orientations[i-1], "WHITE");
		redrawPegs(pegs);
		redrawObst(obst);
		drawRobot(orientations[i], "BLACK");
		pause(PAUSE_TIME);
	}
}

Vector<Orientation> PRM(Node & nextNode, Orientation & robot, Vector<GPoint> & obst){
	Vector<Orientation> retval;
	Vector<Node> sample = createNodeVec(randomSample(nextNode.point, robot));
	sample = trimSample(sample, robot, obst);
	sample.add(nextNode);
	
	Vector<Node> prm = genPRMPath(robot.endpt, nextNode, sample, obst, robot);
	if(prm.size() != 0){
		for (int i = 0; i < prm.size(); i++){
			retval.add(inverseKinematics(prm[i].point, robot.anchor, prm[i].flipped));
		}
		retval[0] = robot;
	}	
	return retval;
}

Vector<Node> trimSample(Vector<Node> & sample, Orientation & robot, Vector<GPoint> & obst){
	Orientation o;
	Orientation save = robot;
	foreach(Node n in sample){
		o = inverseKinematics(n.point, robot.anchor, n.flipped);
		if(isColliding(o, obst)) removeNodeFromVec(sample, n);
	}
	return sample;
}

bool isValidPRMPath(Vector<Node> & prmPath, Vector<GPoint> & obst, Orientation & robot){
	Orientation curr = robot;
	for(int i = 0; i < prmPath.size(); i++){
		Orientation o = inverseKinematics(prmPath[i].point, curr.anchor, prmPath[i].flipped);
		Vector<Orientation> smallPath = genSmallPath(curr, o);
		foreach(Orientation o in smallPath){
			if(isColliding(o, obst)) return false;
		}
		curr = o;
	}
	return true;
}

Vector<Node> genPRMPath(GPoint & start, Node & goal, Vector<Node> & nodes, 
					   Vector<GPoint> & obst, Orientation & robot){
	Queue< Vector<Node> > fringe;
	Vector<Node> vec;
	Map< string, Vector<Node> > map;
	Vector<Node> usedNodes;

	Node startNode;
	startNode.point = start;
	startNode.flipped = true;
	vec.add(startNode);

	map.put(nodeVecToString(vec), usedNodes);
	fringe.enqueue(vec);

	while(!fringe.isEmpty()){
		vec = fringe.dequeue();
		if(vec[vec.size() - 1].point == goal.point &&
			vec[vec.size() - 1].flipped == goal.flipped) {
			if(isValidPRMPath(vec, obst, robot)) return vec;
			else {
				continue;
				cout << "not supposed to be in here: genPRMPath" << endl;
			}
		}
		usedNodes = map[nodeVecToString(vec)];
		for (int i = 0; i < nodes.size(); i++){
			if(nodeVecContains(usedNodes, nodes[i])) continue;
			Vector<Node> usedNodes2 = usedNodes;
			Vector<Node> vec2 = vec;
			vec2.add(nodes[i]);
			fringe.enqueue(vec2);
			usedNodes2.add(nodes[i]);

			map.put(nodeVecToString(vec2), usedNodes2);
		}
	}
	cout << "no PRM path" << endl;
	vec.clear();
	return vec;
}

//Vector<Orientation> choosePath(Vector<GPoint> & samplePoints, Orientation & robot, GPoint & goal, Vector<GPoint> & obst){
//	Vector<Orientation> retval;
//	GPoint lastPoint = robot.endpt;
//	GPoint p = getClosestPt(samplePoints, lastPoint);
//	while(p.getX() != goal.getX() && p.getY() != goal.getY()){
//		Orientation possOrient = inverseKinematics(p, robot);
//		if(isColliding(possOrient, obst)) removePtFromVec(samplePoints, p);
//		else {
//			retval.add(possOrient);
//			lastPoint = p;
//		}
//		p = getClosestPt(samplePoints, lastPoint);
//	}
//	return retval;
//}

void removePtFromVec(Vector<GPoint> & vec, GPoint & pt){
	int x = 0;
	for(int i = 0; i < vec.size(); i++){
		if (vec[i] == pt) {
			x = i;
			break;
		}
	}
	vec.removeAt(x);
}

void removeNodeFromVec(Vector<Node> & vec, Node & n){
	int x = 0;
	for(int i = 0; i < vec.size(); i++){
		if (vec[i].point == n.point &&
			vec[i].flipped == n.flipped) {
			x = i;
			break;
		}
	}
	vec.removeAt(x);
}

//GPoint getClosestPt(Vector<GPoint> & samplePoints, GPoint & lastPoint){
//	GPoint retval = GPoint(9999,9999);
//	foreach(GPoint p in samplePoints){
//		if (distance(p.getX(), p.getY(), lastPoint.getX(), lastPoint.getY()) <
//			distance(p.getX(), p.getY(), retval.getX(), retval.getY()))
//			retval = p;
//	}
//	return retval;
//}

Vector<GPoint> getPegsInRange(Vector<GPoint> & pegs, GPoint & anchor){
	Vector<GPoint> retval;
	foreach(GPoint p in pegs){
		if(p != anchor && inRange(p, anchor)) retval.add(p);     
	}
	return retval;
}

bool inRange(GPoint & p1, GPoint & p2){
	return distance(p1.getX(), p1.getY(), p2.getX(), p2.getY()) <= ROBOT_LENGTH*2;
}

Vector<GPoint> randomSample(GPoint & origin, Orientation & robot){
	Vector<GPoint> retval;
	for(int i = 0; i < 3; i++){
		double x = randomReal(origin.getX() - ROBOT_LENGTH, origin.getX() + ROBOT_LENGTH);
		double y = randomReal(origin.getY() - ROBOT_LENGTH, origin.getY() + ROBOT_LENGTH);

		if(inRange(GPoint(x,y), origin)) retval.add(GPoint(x,y));
	}
	return retval;
}

Orientation inverseKinematics(GPoint & nextPt, GPoint & anchor, bool flag){
	Orientation retval;
	double d = distance(nextPt.getX(), nextPt.getY(), anchor.getX(), anchor.getY());
	
	double deltaX = nextPt.getX()- anchor.getX();
	double deltaY = nextPt.getY() - anchor.getY();
	double k = d/2;

	double jointX, jointY;

	if(flag){
		jointX = anchor.getX() + (deltaX/2) + ((deltaY/d)*sqrt(ROBOT_LENGTH*ROBOT_LENGTH - k*k));
		jointY = anchor.getY() + (deltaY/2) - ((deltaX/d)*sqrt(ROBOT_LENGTH*ROBOT_LENGTH - k*k));
	} else {
		jointX = anchor.getX() + (deltaX*k/d) - (deltaY/d)*sqrt(pow(ROBOT_LENGTH,2) - k*k);
		jointY = anchor.getY() + (deltaY*k/d) + (deltaX/d)*sqrt(pow(ROBOT_LENGTH,2) - k*k);
	}
	
	retval.anchor = anchor;
	retval.joint = GPoint(jointX,jointY);
	retval.endpt = nextPt;

	retval.theta1 = atand(-(retval.joint.getY() - retval.anchor.getY())/(retval.joint.getX() - retval.anchor.getX()));
	if (((retval.joint.getX() < retval.anchor.getX()) && (retval.joint.getY() > retval.anchor.getY())) || 
		(retval.joint.getX() < retval.anchor.getX())) retval.theta1 += 180;

	retval.theta2 = atand(-(retval.endpt.getY() - retval.joint.getY())/(retval.endpt.getX() - retval.joint.getX()));
	if (((retval.endpt.getX() < retval.joint.getX()) && (retval.endpt.getY() > retval.joint.getY())) || 
		(retval.endpt.getX() < retval.joint.getX())) retval.theta2 += 180;


	return retval;
}

bool isColliding(Orientation & robot, Vector<GPoint> & obstacles){
	foreach(GPoint obst in obstacles){
		if(distance(robot.joint.getX(), robot.joint.getY(), obst.getX(), obst.getY()) <= OBST_DIAM/2.0) 
			return true;
		if(distance(robot.endpt.getX(), robot.endpt.getY(), obst.getX(), obst.getY()) <= OBST_DIAM/2.0) 
			return true;
	}
	return false;
}

void initialization(Vector<GPoint> & pegs, Vector<GPoint> & obst, GPoint & start, GPoint & goal, GMouseEvent & e){
	inputPegs(pegs, e);
	inputObstacles(obst, e);
	inputStart(start, e);
	pegs.insertAt(0, start);
	inputGoal(goal, e);
	pegs.add(goal);
}

void drawRobot(Orientation & o, string color){
	setColor(color);
	drawLine(o.anchor, o.joint);
	drawLine(o.joint, o.endpt);
}

void drawRobot(GPoint & anchor, double & theta1, double & theta2, string color){
	setColor(color);
	GPoint p = drawPolarLine(anchor, ROBOT_LENGTH, theta1);
	drawPolarLine(p, ROBOT_LENGTH, theta2);
}

void inputPegs(Vector<GPoint> & pegs, GMouseEvent & e){
	message("Click to set pegs.");
	GRectangle rect = GRectangle(0, 0, 30, 30);
	drawRect(rect);
	GPoint p = getClick(e);
	while(!rect.contains(p)){
		drawPeg(p, "GRAY");
		pegs.add(p);
		p = getClick(e);		
	}
}

void drawPeg(GPoint & p, string color){
	setColor(color);
	fillOval(p.getX() - PEG_DIAM/2, p.getY() - PEG_DIAM/2, PEG_DIAM, PEG_DIAM);
}

void redrawPegs(Vector<GPoint> & pegs){
	drawPeg(pegs[0], "GREEN");
	for(int i = 1; i < pegs.size()-1; i++){
		drawPeg(pegs[i], "GRAY");
	}
	drawPeg(pegs[pegs.size()-1], "RED");
}

void inputObstacles(Vector<GPoint> & obst, GMouseEvent & e){
	message("Click to set obstacles.");
	GRectangle rect = GRectangle(0, 0, 30, 30);
	drawRect(rect);
	GPoint p = getClick(e);
	while(!rect.contains(p)){
		drawObst(p);
		obst.add(p);
		p = getClick(e);		
	}
}

void drawObst(GPoint & p){
	setColor("ORANGE");
	fillOval(p.getX() - OBST_DIAM/2, p.getY() - OBST_DIAM/2, OBST_DIAM, OBST_DIAM);
}

void redrawObst(Vector<GPoint> & obst){
	foreach(GPoint p in obst){
		drawObst(p);
	}
}

void inputStart(GPoint & start, GMouseEvent & e){
	message("Choose starting position.");
	start = getClick(e);
	setColor("GREEN");
	fillOval(start.getX() - PEG_DIAM/2, start.getY() - PEG_DIAM/2, PEG_DIAM, PEG_DIAM);
}

void inputGoal(GPoint & goal, GMouseEvent & e){
	message("Choose ending position");
	goal = getClick(e);
	setColor("RED");
	fillOval(goal.getX() - PEG_DIAM/2, goal.getY() - PEG_DIAM/2, PEG_DIAM, PEG_DIAM);
}

void message(string str){
	setColor("WHITE");
	fillRect(getWindowWidth() - 200, getWindowHeight() - 20, 400, 50);
	setColor("BLACK");
	drawString(str, getWindowWidth() - 150, getWindowHeight() - 10);
}

GPoint getClick(GMouseEvent & e){
	waitForClick(e);
	return GPoint(e.getX(), e.getY());
}

string vecToString(Vector<GPoint> & pts){
	string retval = "";
	foreach(GPoint p in pts){
		retval += realToString(p.getX()) + " " + realToString(p.getY()) + '\n';
	}
	return retval;
}

string nodeVecToString(Vector<Node> & nodes){
	string retval = "";
	foreach(Node n in nodes){
		retval += realToString(n.point.getX()) + " " + realToString(n.point.getY());
		if(n.flipped) retval += "f";
		retval += '\n';
	}
	return retval;
}


double acosd(double x){
	return acos(x) * 180 / PI;
}

double asind(double x){
	return asin(x) * 180 / PI;
}

double atand(double x){
	return atan(x) * 180 / PI;
}

double cosd(double x){
	x = x * PI/180;
	return cos(x);
}

double sind(double x){
	x = x * PI/180;
	return sin(x);
}

double distance (double x1, double y1, double x2, double y2){
	return sqrt(pow(x1 - x2 , 2) + pow(y1 - y2, 2));
}