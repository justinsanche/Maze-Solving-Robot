//
//  MazeStack.h
//  MazeGraph
//
//  Created by Joel Groff on 3/19/17.
//  Copyright © 2017 Joel Groff. All rights reserved.
//

#ifndef MazeStack_h
#define MazeStack_h

class MazeStack
{
    private:
        int stack[16];
        int count;
    public:
        MazeStack();
        void Push(int num);
        int Pop();
        int Top();
		void Clear();
};

#endif /* MazeStack_h */
