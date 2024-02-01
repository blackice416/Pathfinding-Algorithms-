#include <array>
#include <chrono>
#include <queue>
#include <SFML/Graphics.hpp>
#include <functional>
#include <iostream>
#include <ostream>
#include <vector>
#include <cmath>
#include <type_traits>
#include <limits>
#include <string>
using namespace std;
//controls how big grid is
constexpr float WIDTH = 1000;
constexpr float HEIGHT = 600;
//sets window to hieght and width of grid
sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "PathFinding ", sf::Style::Close);
sf::Event evnt;
//controls how big the blocks are in grid
const sf::Vector2f BlockSize = { 50, 50 };
const size_t CollumsX = WIDTH / BlockSize.x;
const size_t CollumsY = HEIGHT / BlockSize.y;

const sf::Color OutlineBlockColor = sf::Color::Black;
const sf::Color WallColor = sf::Color::Black;
const sf::Color StartColor = sf::Color::Blue;
const sf::Color EndColor = sf::Color::Red;
const sf::Color PathColor = sf::Color::Green;
constexpr float Infinity = std::numeric_limits<float>::infinity();

struct Node
{
	//defines node
	bool isWall = false;
	bool isVisited = false;
	float FCost;
	float HCost;
	int x;
	int y;
	std::vector<Node*> Neighbours;
	Node* parent;
};

std::vector<Node*> Nodes;
Node* Start;
Node* End;

void initNeighBours(Node* node)
{

	if (node->x > 0) node->Neighbours.push_back(&Nodes[node->y][node->x - 1]);// checks if node is in most left colum and if not means that there is a node more to the left than this one 
	if (node->y > 0) node->Neighbours.push_back(&Nodes[node->y - 1][node->x]);//top node check 
	if (node->x < CollumsX - 1) node->Neighbours.push_back(&Nodes[node->y][node->x + 1]);//right node check 
	if (node->y < CollumsY - 1) node->Neighbours.push_back(&Nodes[node->y + 1][node->x]);

	if (node->y > 0 && node->x > 0) node->Neighbours.push_back(&Nodes[node->y - 1][node->x - 1]);
	if (node->y < CollumsY - 1 && node->x > 0) node->Neighbours.push_back(&Nodes[node->y + 1][node->x - 1]);
	if (node->y > 0 && node->x < CollumsX - 1) node->Neighbours.push_back(&Nodes[node->y - 1][node->x + 1]);
	if (node->y < CollumsY - 1 && node->x < CollumsX - 1) node->Neighbours.push_back(&Nodes[node->y + 1][node->x + 1]);
}

void initDefNode(Node* node, const int& x, const int& y)
{
	// this function creates the node objects and intailizes all of them to null/base values 



	// creates refrence of based node struct x and y variables 
	node->x = x;
	node->y = y;
	node->isWall = false;
	node->isVisited = false;
	//makes a null pointer vallue for  parent node
	node->parent = nullptr;
}

void init()
{
	for (int y = 0; y < CollumsY; y++)
	{
		//for each y value it creates the cor3espnding x values node 
		Nodes.push_back(new Node[CollumsX]);
		for (int x = 0; x < CollumsX; x++)
		{
			initDefNode(&Nodes[y][x], x, y);
		}
	}

	for (int y = 0; y < CollumsY; y++)
		for (int x = 0; x < CollumsX; x++)
		{
			initNeighBours(&Nodes[y][x]);
		}

	Start = &Nodes[0][0];
	End = &Nodes[CollumsY - 1][CollumsX - 1];
}


void AStarAlg()
{
	for (int y = 0; y < CollumsY; y++)
		for (int x = 0; x < CollumsX; x++)
		{
			Nodes[y][x].isVisited = false;
			Nodes[y][x].FCost = INFINITY;
			Nodes[y][x].HCost = INFINITY;
			Nodes[y][x].parent = nullptr;
		}

	auto GetDist = [](Node* P1, Node* P2) { return sqrt(pow(P2->x - P1->x, 2) + pow(P2->y - P1->y, 2)); };

	Node* CurrentNode = Start;
	Start->HCost = 0.0f;
	Start->FCost = GetDist(Start, End);

	std::vector<Node*> NodesToTest;
	NodesToTest.push_back(Start);

	while (!NodesToTest.empty() && CurrentNode != End)
	{
		std::sort(NodesToTest.begin(), NodesToTest.end(), [](const Node* a, const Node* b) { return a->FCost < b->FCost; });

		while (!NodesToTest.empty() && NodesToTest.front()->isVisited)
			NodesToTest.erase(NodesToTest.begin());

		if (NodesToTest.empty())
			break;

		CurrentNode = NodesToTest.front();
		CurrentNode->isVisited = true;


		for (auto nodeNeighbour : CurrentNode->Neighbours)
		{
			if (!nodeNeighbour->isVisited && !nodeNeighbour->isWall)
				NodesToTest.push_back(nodeNeighbour);

			auto BestNode = CurrentNode->HCost + GetDist(CurrentNode, nodeNeighbour);

			if (BestNode < nodeNeighbour->HCost)
			{
				nodeNeighbour->parent = CurrentNode;
				nodeNeighbour->HCost = BestNode;

				nodeNeighbour->FCost = nodeNeighbour->HCost + GetDist(nodeNeighbour, End);
			}
		}
	}
}


void Dijkstra()
{
	for (int y = 0; y < CollumsY; y++)
		for (int x = 0; x < CollumsX; x++)
		{
			Nodes[y][x].isVisited = false;
			Nodes[y][x].FCost = INFINITY;
			Nodes[y][x].parent = nullptr;
		}

	auto GetDist = [](Node* P1, Node* P2) { return sqrt(pow(P2->x - P1->x, 2) + pow(P2->y - P1->y, 2)); };

	Node* CurrentNode = Start;
	Start->FCost = 0.0f;

	std::vector<Node*> NodesToTest;
	NodesToTest.push_back(Start);

	while (!NodesToTest.empty() && CurrentNode != End)
	{
		std::sort(NodesToTest.begin(), NodesToTest.end(), [](const Node* a, const Node* b) { return a->FCost < b->FCost; });

		while (!NodesToTest.empty() && NodesToTest.front()->isVisited)
			NodesToTest.erase(NodesToTest.begin());

		if (NodesToTest.empty())
			break;

		CurrentNode = NodesToTest.front();
		CurrentNode->isVisited = true;

		for (auto nodeNeighbour : CurrentNode->Neighbours)
		{
			if (!nodeNeighbour->isVisited && !nodeNeighbour->isWall)
				NodesToTest.push_back(nodeNeighbour);

			auto BestNode = CurrentNode->FCost + GetDist(CurrentNode, nodeNeighbour);

			if (BestNode < nodeNeighbour->FCost)
			{
				nodeNeighbour->parent = CurrentNode;
				nodeNeighbour->FCost = BestNode;
			}
		}
	}
}

void BFSAlg()
{
	for (int y = 0; y < CollumsY; y++)
		for (int x = 0; x < CollumsX; x++)
		{
			Nodes[y][x].isVisited = false;
			Nodes[y][x].parent = nullptr;
		}

	std::queue<Node*> NodesToTest;
	NodesToTest.push(Start);
	Start->isVisited = true;

	while (!NodesToTest.empty())
	{
		Node* CurrentNode = NodesToTest.front();
		NodesToTest.pop();

		if (CurrentNode == End)
			break;

		for (auto nodeNeighbour : CurrentNode->Neighbours)
		{
			if (!nodeNeighbour->isVisited && !nodeNeighbour->isWall)
			{
				NodesToTest.push(nodeNeighbour);
				nodeNeighbour->isVisited = true;
				nodeNeighbour->parent = CurrentNode;
			}
		}
	}
}

void measureAlgorithmTime(std::function<void()> algorithm, const std::string& algorithmName)
{
	auto start = std::chrono::steady_clock::now();
	algorithm(); // Execute the algorithm
	auto end = std::chrono::steady_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Algorithm: " << algorithmName << std::endl;
	std::cout << "Length: " << algorithmName.size() << " characters" << std::endl;
	std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
	std::cout << "----------------------------------" << std::endl;
}

void update()
{
	static bool NeedsToBeUpdated = true;

	int x = sf::Mouse::getPosition(window).x / BlockSize.x;
	int y = sf::Mouse::getPosition(window).y / BlockSize.y;

	if (x >= 0 && x < CollumsX && y >= 0 && y < CollumsY)
	{
		if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
		{
			if (!Nodes[y][x].isWall && (x != End->x || y != End->y)) { Start = &Nodes[y][x]; NeedsToBeUpdated = true; }
		}
		if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
		{
			if (!Nodes[y][x].isWall && (x != Start->x || y != Start->y)) { End = &Nodes[y][x]; NeedsToBeUpdated = true; }
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
		{
			static int xPrev = x;
			static int yPrev = y;

			if ((x != xPrev || y != yPrev) && (x != End->x || y != End->y) && (x != Start->x || y != Start->y))
				Nodes[y][x].isWall = !Nodes[y][x].isWall;

			xPrev = x;
			yPrev = y;

			NeedsToBeUpdated = true;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::J))
		{
			measureAlgorithmTime(Dijkstra, "Dijkstra Algorithm");
			measureAlgorithmTime(AStarAlg, "A* Algorithm");
			measureAlgorithmTime(BFSAlg, "Breadth-First Search Algorithm");
		}
	}

	//if (NeedsToBeUpdated) { AStarAlg(); NeedsToBeUpdated = false; }
	//if (NeedsToBeUpdated) { Dijkstra(Start); NeedsToBeUpdated = false; }
	//if (NeedsToBeUpdated) { BFSAlg(); NeedsToBeUpdated = false; }
}




void draw()
{
	for (size_t y = 0; y < CollumsY; y++)
	{
		for (size_t x = 0; x < CollumsX; x++)
		{
			sf::RectangleShape s;
			s.setSize(BlockSize);
			s.setPosition(x * BlockSize.x, y * BlockSize.y);

			if (Nodes[y][x].isWall) s.setFillColor(WallColor);

			if (y == End->y && x == End->x) s.setFillColor(EndColor);

			if (y == Start->y && x == Start->x) s.setFillColor(StartColor);

			s.setOutlineThickness(BlockSize.x / 15);
			s.setOutlineColor(OutlineBlockColor);

			window.draw(s);
		}
	}

	if (End != nullptr)
	{
		Node* p = End->parent;
		while (p != nullptr && p != Start)
		{
			sf::RectangleShape s;
			s.setSize({ BlockSize.x / 2, BlockSize.y / 2 });
			s.setFillColor(PathColor);
			s.setPosition(p->x * BlockSize.x + BlockSize.x / 4, p->y * BlockSize.y + BlockSize.y / 4);

			p = p->parent;

			window.draw(s);
		}
	}

}





int main()
{
	window.setFramerateLimit(60);

	init();

	while (window.isOpen())
	{
		while (window.pollEvent(evnt))
		{
			if (evnt.type == sf::Event::Closed) window.close();
		}

		update();

		window.clear(sf::Color::White);
		//measureFunction(AStarAlg, "AStarAlg");
		//measureFunction(BFSAlg, "BFSAlg");
		//measureFunction(Dijkstra, "Dijkstra");

		draw();

		window.display();
	}
}
