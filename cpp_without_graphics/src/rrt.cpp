#include <iostream>
#include <chrono> // Necessário para medir o tempo
#include <cstdio>
#include <random>
#include <SFML/Graphics.hpp>
#include "geometry.h"
#include <fstream> // Necessário para manipulação de arquivos

using namespace std;

const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

// const double JUMP_SIZE = (fieldWidth / 100.0 * fieldHeight / 100.0) / 1.5;
const double JUMP_SIZE = 300;
const double DISK_SIZE = 100; // Ball radius around which nearby points are found

int printEachIter = 50;
int whichRRT = 1;

vector<Obstacle> obstacles;
Point start, stop;

vector<Point> nodes;
vector<int> parent, nearby;
vector<double> cost, jumps;
int nodeCnt = 0, goalIndex = -1;

vector<sf::ConvexShape> drawablePolygons;
sf::CircleShape startingPoint, endingPoint;
bool pathFound = 0;

Configuration getConfig1()
{
	// ---------- Configuração 1 ------------
	vector<Robot> team = {
			Robot(0, -2050, 0, 0.0),
			Robot(1, -1000, -750, 0.0),
			Robot(2, -1000, 750, 0.0),
	};

	vector<Robot> enemies = {
			Robot(0, 2050, 0, 0.0),
			Robot(1, 1000, 750, 0.0),
			Robot(2, 1000, -750, 0.0),
	};

	return Configuration(team, enemies);
}

Configuration getConfig2()
{
	// ---------- Configuração 2 ------------
	vector<Robot> team = {
			Robot(0, -2000, 0, 0.0),
			Robot(1, -1000, 750, 0.0),
			Robot(2, 0, -750, 0.0),
	};

	vector<Robot> enemies = {
			Robot(0, 2000, 0, 0.0),
			Robot(1, 1250, 750, 0.0),
			Robot(2, 1250, -750, 0.0),
	};

	return Configuration(team, enemies);
}

Configuration getConfig3()
{
	// ---------- Configuração 3 ------------
	vector<Robot> team = {
			Robot(0, -2000, 0, 0.0),
			Robot(1, -1000, 0, 0.0),
			Robot(2, 1000, 0, 0.0),
	};

	vector<Robot> enemies = {
			Robot(0, 2000, 0, 0.0),
			Robot(1, 1250, 750, 0.0),
			Robot(2, 1250, -750, 0.0),
	};

	return Configuration(team, enemies);
}

void getInput()
{

	int currentRobotID = 0;

	Configuration currentConfig = getConfig1();

	fieldWidth = fieldWidth;
	fieldHeight = fieldHeight;
	fieldRadius = fieldRadius;

	int pointsPadding = fieldRadius * 2;
	cout << "Points padding: " << pointsPadding << endl;

	start.x = pointsPadding;
	start.y = fieldHeight / 2;

	stop.x = fieldWidth / 2 - pointsPadding;
	stop.y = fieldHeight / (2 * 2);

	for (const auto &robot : currentConfig.team)
	{
		if (robot.id != currentRobotID)
		{
			if (DEBUG)
				cout << "Adding team robot with ID = " << robot.id << " as a obstacle" << endl;
			Obstacle robotAsObstacle = robot.getObstacle();
			obstacles.push_back(robotAsObstacle);
		}
		else
		{
			start.x = robot.x;
			start.y = robot.y;
		}
	}

	cout << "Starting point: " << start.x << ", " << start.y << endl;
	cout << "Ending point: " << stop.x << ", " << stop.y << endl;

	for (const auto &robot : currentConfig.enemies)
	{
		if (DEBUG)
			cout << "Adding enemy robot with ID = " << robot.id << " as a obstacle" << endl;
		Obstacle robotAsObstacle = robot.getObstacle();
		obstacles.push_back(robotAsObstacle);
	}
}

template <typename T> // Returns a random number in [low, high]
T randomCoordinate(T low, T high)
{
	random_device random_device;
	mt19937 engine{random_device()};
	uniform_real_distribution<double> dist(low, high);
	return dist(engine);
}

// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(Point a, Point b)
{
	for (auto &poly : obstacles)
		if (lineSegmentIntersectsObstacle(a, b, poly))
			return false;
	return true;
}

// Returns a random point with some bias towards goal
Point pickRandomPoint()
{
	double random_sample = randomCoordinate(0.0, 1.0);
	if ((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound)
		return stop + Point(fieldRadius, fieldRadius);
	return Point(randomCoordinate(0, fieldWidth), randomCoordinate(0, fieldHeight));
}

void checkDestinationReached()
{
	sf::Vector2f position = endingPoint.getPosition();

	if (checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(position.x, position.y), fieldRadius))
	{

		pathFound = 1;
		goalIndex = nodeCnt - 1;
		cout << "Reached!! With a distance of " << cost.back() << " units. Path found with a radius of " << fieldRadius << endl
				 << endl;
	}
}

/* Inserts nodes on the path from rootIndex till Point q such
	 that successive nodes on the path are not more than
	 JUMP_SIZE distance away */
void insertNodesInPath(int rootIndex, Point &q)
{
	Point p = nodes[rootIndex];
	if (!isEdgeObstacleFree(p, q))
		return;
	while (!(p == q))
	{
		Point nxt = p.steer(q, JUMP_SIZE);
		nodes.push_back(nxt);
		parent.push_back(rootIndex);
		cost.push_back(cost[rootIndex] + distance(p, nxt));
		rootIndex = nodeCnt++;
		p = nxt;
	}
}

/*  Rewires the parents of the tree greedily starting from
	the new node found in this iterationsation as the parent */
void rewire()
{
	int lastInserted = nodeCnt - 1;
	for (auto nodeIndex : nearby)
	{
		int par = lastInserted, cur = nodeIndex;

		// Rewire parents as much as possible (greedily)
		while (((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS)
		{
			int oldParent = parent[cur];
			parent[cur] = par;
			cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);
			par = cur, cur = oldParent;
		}
	}
}

/*	Runs one iteration of RRT depending on user choice
	At least one new node is added on the screen each iteration. */
void RRT()
{
	Point newPoint, nearestPoint, nextPoint;
	bool updated = false;
	// int cnt = 0;
	int nearestIndex = 0;
	double minCost = INF;
	nearby.clear();
	jumps.resize(nodeCnt);

	while (!updated)
	{
		newPoint = pickRandomPoint();

		// Find nearest point to the newPoint such that the next node
		// be added in graph in the (nearestPoint, newPoint) while being obstacle free
		nearestPoint = *nodes.begin();
		nearestIndex = 0;
		for (int i = 0; i < nodeCnt; i++)
		{
			if (pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while
				cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);

			// Make smaller jumps sometimes to facilitate passing through narrow passages
			jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE;
			auto pnt = nodes[i];
			if ((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i])))
				nearestPoint = pnt, nearestIndex = i;
		}
		nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
		if (!isEdgeObstacleFree(nearestPoint, nextPoint))
			continue;

		if ((whichRRT == 1) or (!pathFound and whichRRT == 3))
		{
			// This is where we don't do any RRT* optimization part
			updated = true;
			nodes.push_back(nextPoint);
			nodeCnt++;
			parent.push_back(nearestIndex);
			cost.push_back(cost[nearestIndex] + distance(nearestPoint, nextPoint));
			if (!pathFound)
				checkDestinationReached();
			continue;
		}

		// Find nearby nodes to the new node as center in ball of radius DISK_SIZE
		for (int i = 0; i < nodeCnt; i++)
			if ((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS and isEdgeObstacleFree(nodes[i], nextPoint))
				nearby.push_back(i);

		// Find minimum cost path to the new node
		int par = nearestIndex;
		minCost = cost[par] + distance(nodes[par], nextPoint);
		for (auto nodeIndex : nearby)
		{
			if (((cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
				minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
		}

		parent.push_back(par);
		cost.push_back(minCost);
		nodes.push_back(nextPoint);
		nodeCnt++;
		updated = true;
		if (!pathFound)
			checkDestinationReached();
		rewire();
	}
}

void saveResultsToFile(const vector<Point> &nodes, const vector<Point> &finalPath, const string &filename)
{
	ofstream file(filename);
	if (file.is_open())
	{
		// Salva todos os pontos gerados
		file << "Todos os pontos gerados:\n";
		for (const auto &node : nodes)
		{
			file << node.x << ", " << node.y << "\n";
		}

		// Salva o caminho final
		file << "\nCaminho final:\n";
		for (const auto &point : finalPath)
		{
			file << point.x << ", " << point.y << "\n";
		}

		file.close();
	}
	else
	{
		cerr << "Não foi possível abrir o arquivo para escrita.\n";
	}
}

int main()
{
	cout << "Configurações do RRT:" << endl
			 << "Tamanho do campo:" << endl
			 << "Largura: " << fieldWidth << endl
			 << "Altura: " << fieldHeight << endl
			 << "Raio do círculo: " << fieldRadius << endl
			 << "Quantidade de obstáculos: " << obstacles.size() << endl
			 << "Tamanho do passo:" << JUMP_SIZE << endl
			 << "Raio do disco para procura do alvo:" << DISK_SIZE << endl
			 << endl;
	getInput();
	// drawInput();

	// sf::RenderWindow window(sf::VideoMode(fieldWidth, fieldHeight), "Basic Anytime RRT");

	nodeCnt = 1;
	nodes.push_back(start);
	int iterations = 0;
	parent.push_back(0);
	cost.push_back(0);

	cout << endl
			 << "Starting point: " << start.x << ", " << start.y << endl
			 << "Ending point: " << stop.x << ", " << stop.y << endl
			 << endl;

	auto start = std::chrono::high_resolution_clock::now();

	// Create a vector of points to represent the final path
	vector<Point> finalPath;

	while (!pathFound)
	{
		RRT();

		iterations++;

		if (iterations % printEachIter == 0)
		{
			cout << "Iterations: " << iterations << endl;
			if (!pathFound)
				cout << "Not reached yet :( " << endl;
			else
				cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl;
		}
	}

	// Finding the final path

	cout << "Path found. Index: " << goalIndex << " Distance: " << cost[goalIndex] << " units. Amount of nodes: " << nodes.size() << endl;
	int node = goalIndex;

	while (parent[node] != node)
	{
		finalPath.push_back(nodes[node]);
		node = parent[node];
	}
	finalPath.push_back(nodes[node]);

	auto end = std::chrono::high_resolution_clock::now();

	// Imprime a duração em segundos
	std::chrono::duration<double> duration = end - start;
	std::cout << "Tempo de execução: " << duration.count() << " segundos" << std::endl;

	// Armazena o caminho final invertido e com os parametros x,y e angle
	vector<Point> finalPathInverted;
	// finalPathInverted.reserve(finalPath.size());
	for (int i = finalPath.size() - 1; i >= 0; i--)
	{
		double angle = atan2(finalPath[i].y - finalPath[i + 1].y, finalPath[i].x - finalPath[i - 1].x);
		Point currentPoint = Point(finalPath[i].x, finalPath[i].y, angle);
		finalPathInverted.push_back(currentPoint);
	}

	// Final Path in a format {x, y, angle}
	cout << "Final path in a format {x, y, angle}: " << endl;
	for (int i = finalPath.size() - 1; i >= 0; i--)
	{
		double angle = atan2(finalPath[i].y - finalPath[i - 1].y, finalPath[i].x - finalPath[i - 1].x);
		cout << "{"
				 << "x: " << finalPath[i].x << ", "
				 << "y: " << finalPath[i].y << ", "
				 << "angle: " << angle << "},"
				 << endl;
	}

	cout << "Final path correct form (reversed)" << endl;
	// Imprime o caminho final invertido

	for (int i = 0; i < int(finalPathInverted.size() - 1); i++)
	{
		cout << "{ x: " << finalPathInverted[i].x << ", y: " << finalPathInverted[i].y << ", angle: " << finalPathInverted[i].angle << " }" << endl;
	}

	// Salva os resultados em um arquivo
	// saveResultsToFile(nodes, finalPath, "RRT_Results.txt");
	// TODO -> Enviar caminho aqui

	return 0;
}