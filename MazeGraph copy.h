//
//  MazeGraph.h
//  MazeGraph
//
//  Created by Joel Groff on 3/14/17.
//  Copyright © 2017 Joel Groff. All rights reserved.
//  Simple graph built for lab3 for USF Robotics
//

#ifndef MazeGraph_h
#define MazeGraph_h

#include <stdio.h>


class MazeGraph
{
    private:
        int size;
        int edgeNum;
        bool cellList[16];   // 0 visited -- 1 unvisited list
		bool cellListBFS[16];
        int edgeList[64][2];
		int parentList[16];
    public:
        MazeGraph();
        void AddEdge(int cell1, int cell2);
        int* GetNeighbors(int cell);
        bool IsVisited(int cell);
        void SetVisited(int cell);
		void SetParentBFS(int cell, int p);
		void SetVisitedBFS(int cell, bool v);
		int GetParentBFS(int cell);
		bool GetVisitedBFS(int cell);
};



#endif /* MazeGraph_h */
