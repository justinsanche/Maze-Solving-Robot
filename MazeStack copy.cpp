//
//  MazeStack.cpp
//  MazeGraph
//
//  Created by Joel Groff on 3/19/17.
//  Copyright © 2017 Joel Groff. All rights reserved.
//

#include "MazeStack.h"

MazeStack::MazeStack()
{
    count = 0;
    for (int i = 0; i < 16; i++)
    {
        stack[0] = -1;
    }
}

void MazeStack::Clear()
{
	count = 0;
	for (int i = 0; i < 16; i++)
	{
		stack[0] = -1;
	}
}

void MazeStack::Push(int num)
{
    if (count >= 16)
    {
        count = 16;
        return;
    }
    
    stack[count] = num;
    count++;
}

int MazeStack::Pop()
{
    if (count<=0)
    {
        count = 0;
        return -1;
    }
    count--;
    return stack[count];
}

int MazeStack::Top()
{
    if (count<=0)
    {
        count = 0;
        return -1;
    }
    return stack[count - 1];
}
