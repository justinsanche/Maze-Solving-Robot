//
//  MazeGraph.cpp
//  MazeGraph
//
//  Created by Joel Groff on 3/14/17.
//  Copyright © 2017 Joel Groff. All rights reserved.
//

#include "MazeGraph.h"

MazeGraph::MazeGraph()
{
    size = 16;
    edgeNum = 0;

    // initialize all edges to -1
    for (int row = 0; row < 64; row++)
    {
        for (int col = 0; col < 2; col++)
        {
            edgeList[row][col] = -1;
        }
    }

    // initialize all cells to unvisited
    for (int i = 0; i < size; i++)
    {
        cellList[i] = false;
		cellListBFS[i] = false;
		parentList[i] = -1;
    } 
}

void MazeGraph::AddEdge(int cell1, int cell2)
{
    if (edgeNum >= 64)
    {
        return;
    }
    // If edge already exists
    for (int i = 0; i < 64; i++)
    {
        if ((edgeList[i][0] == cell1 && edgeList[i][1] == cell2)
            || (edgeList[i][1] == cell1 && edgeList[i][0] == cell2))
            {
                return;
            }
    }
    edgeList[edgeNum][0] = cell1;
    edgeList[edgeNum][1] = cell2;
    edgeNum++;
}

int* MazeGraph::GetNeighbors(int cell)
{
    int* tempArray = new int[4]();

    int k = 0;
    for (k = 0; k < 4; k++)
        tempArray[k] = -1;
    k = 0;

    for (int row = 0; row < 64; row++)
    {
        for (int col = 0; col < 2; col++)
        {
            if (edgeList[row][col] == cell)
            {
                if (col == 0)
                {
                    tempArray[k] = edgeList[row][1];
                    k++;
                }
                else
                {
                     tempArray[k] = edgeList[row][0];
                     k++;
                }
            }
        }
    }
    return tempArray;
}

bool MazeGraph::IsVisited(int cell)
{
    return cellList[cell];
}

void MazeGraph::SetVisited(int cell)
{
    cellList[cell] = true;
}

void MazeGraph::SetVisitedBFS(int cell, bool v)
{
	cellListBFS[cell] = v;
}

void MazeGraph::SetParentBFS(int cell, int p)
{
	parentList[cell] = p;
}

bool MazeGraph::GetVisitedBFS(int cell)
{
	return cellListBFS[cell];
}

int MazeGraph::GetParentBFS(int cell)
{
	return parentList[cell];
}
