#include "AStar.h"
#include <list>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::list<NodeRecord> openList{};
	std::list<NodeRecord> closedList{};
	NodeRecord currentNodeRecord{};
	NodeRecord startNodeRecord{};
	
	startNodeRecord.pNode = pStartNode;
	startNodeRecord.pConnection = nullptr;
	startNodeRecord.costSoFar = 0;
	startNodeRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	openList.push_back(startNodeRecord);
	
	while (!openList.empty())
	{
		currentNodeRecord = *std::min_element(openList.begin(), openList.end());
		
		if (currentNodeRecord.pNode == pGoalNode) break;
		
		const auto& connections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());
		for (const auto& connection : connections)
		{
			Node* pNextNode = pGraph->GetNode(connection->GetToId()).get();
			float totalCostSoFar = currentNodeRecord.costSoFar + connection->GetWeight();
			
			auto closedNodeIt = std::find_if(closedList.begin(), closedList.end(), [pNextNode](const NodeRecord& record) { return record.pNode == pNextNode; });
			
			if (closedNodeIt != closedList.end() && closedNodeIt->costSoFar <= totalCostSoFar)
			{
				continue;
			}
			if (closedNodeIt != closedList.end())
			{
				closedList.erase(closedNodeIt);
			}
			
			auto openNodeIt = std::find_if(openList.begin(), openList.end(), [pNextNode](const NodeRecord& record) { return record.pNode == pNextNode; });

			if (openNodeIt != openList.end() && openNodeIt->costSoFar <= totalCostSoFar)
			{
				continue;
			}
			if (openNodeIt != openList.end())
			{
				openList.erase(openNodeIt);
			}
			
			NodeRecord newRecord;
			newRecord.pNode = pNextNode;
			newRecord.pConnection = connection;
			newRecord.costSoFar = totalCostSoFar;
			newRecord.estimatedTotalCost = totalCostSoFar + GetHeuristicCost(pNextNode, pGoalNode);
			openList.push_back(newRecord);
		}
		
		openList.remove(currentNodeRecord);
		closedList.push_back(currentNodeRecord);
	}
	
	// Backtracking
	while (currentNodeRecord.pNode != pStartNode)
	{
		path.push_back(currentNodeRecord.pNode);
		auto closedNodeIt = std::find_if(closedList.begin(), closedList.end(), [&currentNodeRecord](const NodeRecord& record) { return record.pNode->GetId() == currentNodeRecord.pConnection->GetFromId(); });
		currentNodeRecord = *closedNodeIt;
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());
	
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}