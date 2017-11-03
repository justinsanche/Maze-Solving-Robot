
  // Joel Groff, Justin Sanchez
  // USF Mobile Robots
  // Lab 4 Task 1 & 2
  // The robot uses this program to navigate a 4 x 4 maze, building the wall locations as it moves. 
  
  #include <Servo.h>
  #include <Adafruit_RGBLCDShield.h>
  #include <utility/Adafruit_MCP23017.h>
  #include <MazeGraph.h>
  #include <math.h>
  #include <MazeStack.h>
  
  Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
  
  const int SFSensor = A0;
  const int SLSensor = A1;
  const int SRSensor = A2;
  const int LFSensor = A3;
  
  Servo LServo;
  Servo RServo;
  
  // Encoders
  int encPinR = 11;
  int encPinL = 10;
  
  int lastLeftRead = 0;
  int lastRightRead = 0;
  int currentLeftRead = 0;
  int currentRightRead = 0;

  float currentDegrees = 0; // The theoretical direction of robot
  float actualDegrees = 0; //  The actual direction of robot 
  
  // Number holes read by encoders
  int leftCount = 0;
  int rightCount = 0;
  
  // Colors
  #define RED 0x1
  #define YELLOW 0x3
  #define GREEN 0x2
  #define TEAL 0x6
  #define BLUE 0x4
  #define VIOLET 0x5
  #define WHITE 0x7
  
  //color sensor
  #define S0 4
  #define S1 5
  #define S2 6
  #define S3 7
  #define sensorOut 8
  
  // Color reading variables
  int red_read = 0;
  int blue_read = 0;
  int green_read = 0;
  int countBlue = 0;
  int countRed = 0;
  int red_ave = 0;
  int blue_ave = 0;
  int none_ave = 0;
  enum Color {red, blue, none};
  Color color;  // Color detected from reading color sensors
  
  // Sensor reading
  int sensor_val = 0; // Analogue reading
  float short_dist = 0; // SF sensor in inches
  float long_dist = 0; // LF sensor in inches
  float comb_dist = 0; // Combined front in inches
  float left_dist = 0; // SL sensor in inches
  float right_dist = 0; // SR sensor in inches

  float last_left_dist = 0;
  float last_right_dist = 0;
  
  // Variables used to average sensor values
  float short_read[10];
  float long_read[10];
  float left_read[10];
  float right_read[10];
  int read_count = 0;
  
  // Time variables
  unsigned long current_time;
  unsigned long dist_timer;   // How long between dist reads
  unsigned long color_timer;  // How long between color reads
  unsigned long final_color_timer; // How long between color state change
  unsigned long start_time;
  unsigned long display_timer;
  unsigned long encoder_timer;
  unsigned long centerTimer;
  unsigned long resetTimer;
  unsigned long menuTimer;
  
  // Wheel velocity
  int rightVel = 90;
  int leftVel = 90;
  
  // Robot States
  enum GlobalState {FORWARD, BACKWARD, START, STOP, PATH, MENU}; // Moving toward an adjacent or backtracking
  GlobalState globalState;
  enum LocalState {TURN_RIGHT, TURN_LEFT, TURN_AROUND, STRAIGHT}; // Specific behavior
  LocalState localState;
  
  int targetCell;
  int currentCell;
  
  enum Wall{NA, YES, NO};
  Wall wallN;
  Wall wallE;
  Wall wallS;
  Wall wallW;
  
  // Maze
  MazeGraph maze;
  MazeStack mazeStack;
  
  bool startTowardCenterFlag; // Flag that the robot has started moving toward the center of a cell
  int towardCenterEncoderCount; // What the encoders were at when the toward center flag was set

  int GOAL;
  uint8_t buttons;
  int select;
  int orientation;
  
  void setup() {
      LServo.attach(2);
      RServo.attach(3);
      Serial.begin(9600);
      pinMode(encPinR, INPUT_PULLUP);
      pinMode(encPinL, INPUT_PULLUP);
  
       //color sensor
      pinMode(S0, OUTPUT);
      pinMode(S1, OUTPUT);
      pinMode(S2, OUTPUT);
      pinMode(S3, OUTPUT);
      pinMode(sensorOut, INPUT);
    
      // Setting frequency-scaling to 20%
      digitalWrite(S0,HIGH);
      digitalWrite(S1,LOW);

      // Set up the LCD's columns and rows
      lcd.begin(16,2);
      lcd.setCursor(1,6);
      lcd.setBacklight(YELLOW);
  
      // Reset timers
      current_time = millis();
      color_timer = millis();
      final_color_timer = millis();
      dist_timer = millis();
      start_time = millis();
      display_timer = millis();
      encoder_timer = millis();
      centerTimer = 0;
      resetTimer = millis();
      menuTimer = millis();
      
      // Set initial color
      color = none;
      
      lastLeftRead = digitalRead(encPinL);
      lastRightRead = digitalRead(encPinR);
      currentLeftRead = lastLeftRead;
      currentRightRead = lastRightRead;

      leftCount = 0;
      rightCount = 0;
      rightVel = 90;
      leftVel = 90;
  
      globalState = MENU;
      localState = STRAIGHT;

      // Default
      targetCell = 0;
      currentCell = 0;
      GOAL = 0;
      
      wallN = NA;
      wallE = NA;
      wallS = NA;
      wallW = NA;

      startTowardCenterFlag = false;
      towardCenterEncoderCount = 0;

     currentDegrees = 0;
     actualDegrees = 0;

    // Menu stuff
    orientation = 0;
    select = 0;
    GOAL = 0;
  }
  
  void loop() 
  {
      Update_Time();
      Check_Color();
      // 5 second delay for all functionality
      if (current_time - start_time <= 5000)
      {
          Get_Distance_Readings();
      }
      if (current_time - start_time > 5000)
      {
          UpdateEncoders();
          UpdateDegrees();
          Get_Distance_Readings();
          switch(globalState)
          {
              case MENU:
                  Menu();
                  break;
              case START:
                  //for (int i = 0; i < 16; i++)
                  //{
                   //   maze.SetVisited(i);
                  //}
                  CheckWalls();
                  AddWalls();
                  Display();
                  PickTarget();
                  break;
              case FORWARD:
              case BACKWARD:
              case PATH:
                  GlobalLoop();
                   break;
              case STOP:
                  leftVel = 90;
                  rightVel = 90;
                  break;
          }
          
          LServo.write(leftVel);
          RServo.write(rightVel);
      }
      else
      {
          LServo.write(90);
          RServo.write(90);
      }

  }

 void Menu()
  {
    if (Check_Timer(menuTimer) > 200)
    {
          LServo.write(90);
          RServo.write(90);
           
          buttons = lcd.readButtons();
    
          //run function that switches state
          SwitchState();
      
          switch(select)
          {
              case 0:
                  StartState();
                  break;
              case 1:
                  OrientationState();
                  break;
              case 2:
                  EndState();
                  break;
              case 3:
                  ExitState();
                  break;
          }
    }
    
  }



void SwitchState()
{
  if (buttons & BUTTON_LEFT)
  {
    lcd.clear();
    select = select - 1;
    if(select < 0)
      select = 3;
     Set_Timer(menuTimer);
  }
  else if(buttons & BUTTON_RIGHT)
  {
    lcd.clear();
    select = select + 1;
    if(select > 3)
      select = 0;
     Set_Timer(menuTimer);
  }
}

void StartState()
{

  lcd.setCursor(0,0);
  lcd.print("STARTING LOCAT.");
 
    if(buttons & BUTTON_UP)
    {
       lcd.clear();
        currentCell++;
       Set_Timer(menuTimer);
    }
     else if(buttons & BUTTON_DOWN)
     {
        lcd.clear();
        currentCell--;
         Set_Timer(menuTimer);
     }
    if(currentCell < 0)
      currentCell = 15;

    if(currentCell > 15)
      currentCell = 0;

    lcd.setCursor(0,1);
    lcd.print(currentCell);
}

void OrientationState()
{
  lcd.setCursor(0,0);
  lcd.print("ORIENTATION");
 
    if(buttons & BUTTON_UP)
    {
      lcd.clear();
        orientation++;
         Set_Timer(menuTimer);
    }
     else if(buttons & BUTTON_DOWN)
     {
        lcd.clear();
        orientation--;
         Set_Timer(menuTimer);;
     }
    if (orientation > 3)
    {
        orientation = 0;
    }
    if (orientation < 0)
    {
        orientation = 3;
    }
    switch(orientation){
      case 0:
        lcd.setCursor(0,1);
        lcd.print("NORTH");
       break;
      case 1:
        lcd.setCursor(0,1);
        lcd.print("EAST");
       break;
       case 2:
        lcd.setCursor(0,1);
        lcd.print("SOUTH");
       break;
       case 3:
        lcd.setCursor(0,1);
        lcd.print("WEST");
       break;
  }
}

void EndState()
{
  lcd.setCursor(0,0);
  lcd.print("GOAL LOCAT.");
 
    if(buttons & BUTTON_UP)
    {
      lcd.clear();
        GOAL++;
         Set_Timer(menuTimer);
    }
     else if(buttons & BUTTON_DOWN)
     {
      lcd.clear();
        GOAL--;
        Set_Timer(menuTimer);
     }

    if(GOAL < 0)
      GOAL = 0;

    if(GOAL > 15)
      GOAL = 15;

    lcd.setCursor(0,1);
    lcd.print(GOAL);
}

void ExitState()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EXIT");
    if (buttons & BUTTON_SELECT)
    {
      lcd.clear();
          if ( currentCell != GOAL )
          {
              switch (orientation)
              {
                  case 0:
                      leftCount = 0;
                      rightCount = 0;
                      actualDegrees = 0;
                      currentDegrees = 0;
                      break;
                  case 1:
                      leftCount = 52;
                      rightCount = 0;
                      actualDegrees = 270;
                      currentDegrees = 270;
                      break;
                  case 2:
                      leftCount = 0;
                      rightCount = 104;
                      actualDegrees = 180;
                      currentDegrees = 180;
                      break;  
                 case 3:
                      leftCount = 0;
                      rightCount = 52;
                      actualDegrees = 90;
                      currentDegrees = 90;
                      break;
              }
            if (maze.IsVisited(GOAL) == true)
            { 
                 // Find the shortest path and put it on the mazeStack
                BFS();
                // Begin moving
                globalState = PATH;
                localState = STRAIGHT;
               
                mazeStack.Push(currentCell);
                PickTarget();
            }
            else
            {
                localState = STRAIGHT;
                globalState = START;
                mazeStack.Push(currentCell);
                maze.SetVisited(currentCell);
            }
          } 
      select = 0;
    }    
}
  // *****************************************************************
  // GLOBAL MACHINE
  // *****************************************************************

  void GlobalLoop()
  {
      switch(localState)
      {
          case STRAIGHT:
            MoveStraight();
            CheckCell();
            break;
          case TURN_LEFT:
            TurnLeft();
            break;
          case TURN_RIGHT:  
            TurnRight();
            break;
          case TURN_AROUND:
            TurnAround();
            break;
      }
  }

 
  
  // *****************************************************************
  // LOCAL BEHAVIORS
  // *****************************************************************
  
  void CheckWalls()
  {
      bool left = false;
      bool right = false;
      bool straight = false;
  
      if (left_dist <= 11.0f)
          left = true;
      if (right_dist <= 11.0f)
          right = true;
      if (comb_dist <= 15.0f)
          straight = true;
  
      wallN = NA;
      wallS = NA;
      wallE = NA;
      wallW = NA;
  
      // Set walls based on the degree robot is facing
      if (currentDegrees == 0)
      {
          wallN = NO;
          wallW = NO;
          wallE = NO;
          if (straight)
             wallN = YES;
          if (left)
             wallW = YES;
          if (right)
             wallE = YES;
      }
      else if (currentDegrees == 90.0f)
      {
          wallN = NO;
          wallW = NO;
          wallS = NO;
          if (straight)
             wallW = YES;
          if (left)
             wallS = YES;
          if (right)
             wallN = YES;
      }
      else if (currentDegrees == 180.0f)
      {
          wallS = NO;
          wallW = NO;
          wallE = NO;
          if (straight)
             wallS = YES;
          if (left)
             wallE = YES;
          if (right)
             wallW = YES;
      }
      else if (currentDegrees == 270.0f)
      {
          wallN = NO;
          wallS = NO;
          wallE = NO;
          if (straight)
             wallE = YES;
          if (left)
             wallN = YES;
          if (right)
             wallS = YES;
      }
  }
  
  // Add edges to the graph
  void AddWalls()
  {
      if (wallN == NO)
          maze.AddEdge(currentCell, currentCell - 4);
      if (wallS == NO)
          maze.AddEdge(currentCell, currentCell + 4);
      if (wallE == NO)
          maze.AddEdge(currentCell, currentCell + 1);
      if (wallW == NO)
          maze.AddEdge(currentCell, currentCell - 1);
  }
  
  // Figure out our next cell
  void PickTarget()
  {
      if (globalState != PATH)
      {
          int* neighbors = maze.GetNeighbors(currentCell);
          int i = 0;
          
          for (i = 0; i < 4; i++)
          {
              if (neighbors[i] != -1)
              {
                  if (!maze.IsVisited(neighbors[i]))
                  {
                      targetCell = neighbors[i]; 
                      globalState = FORWARD;
                      break;
                  }
              }
          }
          // Set target for backtracking
          if (i >= 4)
          {
              // Uh oh we need to backtrack!
              globalState = BACKWARD;
              mazeStack.Pop();
              targetCell = mazeStack.Top();
              if (targetCell == -1)
                  globalState = STOP;
          }
      }
      // Set target for path 
      else
      {
              mazeStack.Pop();
              targetCell = mazeStack.Top();
      }
     
      if (targetCell == currentCell + 1)
      {
              // Robot needs to move East
              PickDirection(270.0f);
       }
       else if (targetCell == currentCell - 1)
        {
              // Robot needs to move West
              PickDirection(90.0f);
       }
       else if (targetCell == currentCell + 4)
       {
              // Robot needs to move South
              PickDirection(180.0f);
       }
       else
       {
             // Robot needs to move North
             PickDirection(0);
       }
  }

  // Figure out which direction to turn based on current and desired adjacency
  void PickDirection(float desired)
  {
      float relative = desired - currentDegrees;
      if (relative == 0)
      {
          localState = STRAIGHT;
      }
      else if (relative == 90.0f || relative == -270.0f)
      {
          localState = TURN_LEFT;
      }
      else if (relative == -90.0f || relative == 270.0f)
      {
          localState = TURN_RIGHT;
      }
      else
      {
          localState = TURN_AROUND;
      }
  }
  
  // Robot moves forward in a straight line
  void MoveStraight()
  {
      if (resetTimer >= 2000)
      {
        // Reset encoders
        if (left_dist - last_left_dist <= 0.2f && left_dist - last_left_dist >= -0.2)
          {
              if (currentDegrees == 0)
            {
                leftCount = 0;
                rightCount = 0;
            }
            else if (currentDegrees == 90)
            {
                leftCount = 0;
                rightCount = 52;
            }
            else if (currentDegrees == 180)
            {
                leftCount = 0;
                rightCount = 104;
            }
            else
            {
                leftCount = 0;
                rightCount = 156;
            }
  
            last_left_dist = left_dist;
            last_right_dist = right_dist;
            Set_Timer(resetTimer);
          }
      }
      
      if ((actualDegrees < currentDegrees) || (actualDegrees >=340.0f) )
      {
          // Veer left
          leftVel = 90;
          rightVel = 88;
      }
      else if ((actualDegrees > currentDegrees) && (actualDegrees < 340.0f))
      {
          // Veer right
          rightVel = 90;
          leftVel = 92;
      }
      else
      {
          rightVel = 88;
          leftVel = 92;
      }
  
     if (left_dist < 3.0f)
     {
        rightVel = 89;
        leftVel = 92;
     }
     else if (right_dist < 3.0f)
     {
        leftVel = 91;
        rightVel = 88;
     }
     
  }

  
  // Update our location and move to center of cell
  void CheckCell()
  {
       // If we get a color read for the first time
      if ((color == red || color == blue) && startTowardCenterFlag == false)
      {
              // Store current encoder
              Set_Timer(centerTimer);
              startTowardCenterFlag = true;
              currentCell = targetCell;
              
              maze.SetVisited(currentCell);
 
              if (globalState == FORWARD)
              {
                  mazeStack.Push(currentCell);
              }
       }

       // If we already crossed cells, keep moving toward center
       if (startTowardCenterFlag == true)
       {
           // In center of cell
           if ( Check_Timer(centerTimer) >= 5500 )
           { 
                if (globalState == PATH)
                {
                    if (currentCell == GOAL)
                    {
                        globalState = MENU;
                        return;
                    }
                }
                // If we are in a new cell
                if (globalState == FORWARD)
                {
                  CheckWalls();
                  AddWalls();
                }
                
                PickTarget();
    
                // Reset center flags
                startTowardCenterFlag = false;
                Display();
            }
        }
  }
  
  void TurnRight()
  {
      float degree = currentDegrees - actualDegrees;
      if (( degree >= 88.5f && degree <= 91.5f)
          || (degree >=-272.0f && degree <=-267.0f))
      {
          currentDegrees -= 90.0f;
          if (currentDegrees < 0)
            currentDegrees = 270.0f;

           // Done turning set new state
           localState = STRAIGHT;    
      }
      else
      {
          leftVel = 92; 
          rightVel = 91;
      }
  }

  void TurnLeft()
  {
      float degree = actualDegrees - currentDegrees;
      if ( (degree >= 88.0f && degree <= 92.0f) ||
          (degree >=268.0f && degree <=272.0f))
      {
          currentDegrees += 90.0f;
          if (currentDegrees >= 360.0f)
            currentDegrees = 0;
          
          // Done turning set new state
          localState = STRAIGHT;
      }
      else
      {
          leftVel = 89; 
          rightVel = 88;
      }
  }

  void TurnAround()
  {
      float degree = actualDegrees - currentDegrees;
      if (degree < 0)
        degree *= -1;
      if (  degree >= 178.5f && degree <= 181.5f)
        {
            currentDegrees += 180.0f;
            if (currentDegrees == 360.0f)
              currentDegrees = 0;
            else if (currentDegrees == 450.0f)
              currentDegrees = 90.0f;
            // Done turning set new state
            localState = STRAIGHT;
        }
        else
        {
            leftVel = 89; 
            rightVel = 89;
        }
  }
  
  // Turn back to facing zero
  void ResetToZero()
  {
      if (actualDegrees >= 359.0f || actualDegrees <= 1.0f)
      {
          //Switch states
          currentDegrees = 0;
      }
      else
      {
          leftVel = 89;
          rightVel = 89;
      }

  }
  
  
  void BFS()
  {
      mazeStack.Clear();

      
        // Set all values to false first
        for (int i = 0; i < 16; i++)
        {
            maze.SetVisitedBFS(i, false);
        }

        int queue[16];
        for (int i = 0; i < 16; i++)
            queue[i] = -2;
        
        int front = 0;
        int last = 0;
        queue[last] = currentCell;
        maze.SetVisitedBFS(queue[last], true);
        last++;
        while (last < 20)
        {
            int* neighbors = maze.GetNeighbors(queue[front]);
            for (int i = 0; i < 4; i++)
            {
                 if (neighbors[i] != -1)
                 {
                      if (maze.GetVisitedBFS(neighbors[i]) == false)
                      {
                          queue[last] = neighbors[i];
                          maze.SetVisitedBFS(neighbors[i], true);
                          maze.SetParentBFS(neighbors[i], queue[front]);
                          last++;
                          if (neighbors[i] == GOAL)
                          {
                             mazeStack.Push(GOAL);
                             int parent = maze.GetParentBFS(GOAL);
                             while (parent != currentCell)
                             {
                                  mazeStack.Push(parent);
                                  int p = maze.GetParentBFS(parent);
                                  parent = p;
                             }
                              return;
                          }
                      }
                 }
            }
            front++;
        }

  }
  
  
  //****************************************************************************************
  // SENSOR READINGS
  //****************************************************************************************
  
  // Set the robot's actual degrees each loop
 void UpdateDegrees()
  {
      int difference = leftCount - rightCount;
      if (difference < 0)
      {
        difference = difference * (-1);
      }
  
      float degree = (((float)difference) * (90.0 / 52.0));
      float tempDegree = degree;
      if (leftCount > rightCount)
      {
          degree = 360.0f - degree;
      }
  
      if (degree >= 360 || degree <=0)
      {
          degree = 0;
          leftCount = 0;
          rightCount = 0;
      }
      
      actualDegrees = degree;
  }
  
  // Get new encoder values and currentDegrees
  void UpdateEncoders()
  {
      if (Check_Timer(encoder_timer) > 10)
      {
          Set_Timer(encoder_timer);
          currentLeftRead = digitalRead(encPinL);
          currentRightRead = digitalRead(encPinR);
      
          if (currentLeftRead != lastLeftRead)
          {
              if (leftVel >= 90)
                leftCount++;
              else
                leftCount--;
            
              
          }
          if (currentRightRead != lastRightRead)
          {
              
             if (rightVel <= 90)
                rightCount++;
              else
                rightCount--;
              
          } 
          lastLeftRead = currentLeftRead;
          lastRightRead = currentRightRead;
      }
  }
  
  // Changes the screen based on the color beneath the robot, red, blue, and other
  void Check_Color()
  {
      // Gather color info over time
      if (Check_Timer(color_timer) >= 20)
      {
          // Reset timer
          Set_Timer(color_timer);
          
          digitalWrite(S2,LOW);
          digitalWrite(S3,LOW);
          red_read = pulseIn(sensorOut, LOW);
          digitalWrite(S2,LOW);
          digitalWrite(S3,HIGH);
          blue_read = pulseIn(sensorOut, LOW);
          digitalWrite(S2,HIGH);
          digitalWrite(S3,HIGH);
          green_read = pulseIn(sensorOut, LOW);
  
          if(red_read <= 90)
          {
              red_ave++;
          }
          else if(blue_read <= 80 && red_read >40 && green_read>50 )
          {
              blue_ave++;
          }
          else
          {
              none_ave++;
          }
          
      }
      // Set the color
      if (Check_Timer(final_color_timer) > 100)
      {  
          Set_Timer(final_color_timer);
          if (red_ave > blue_ave && red_ave > none_ave)
          {
              if (color != red)
                  countRed++;
              color = red;
          }
          else  if (blue_ave > red_ave && blue_ave > none_ave)
          {
              if (color != blue)
                  countBlue++;
              color = blue;
          } 
          else
            color = none;   
          red_ave = 0;
          blue_ave = 0;
          none_ave = 0;
      }   
          
          if (color == red)
              lcd.setBacklight(RED);
          else if (color == blue)
              lcd.setBacklight(BLUE);
          else
              lcd.setBacklight(YELLOW);
  }
  
  
  // return distance of front sensors using both short and long
  float Front_Combined()
  {
          // FORWARD SENSORS
          //lcd.setCursor(0,0);
          //lcd.print("Forward Sensor");
          //lcd.setCursor(0,1);
         
          if (long_dist > 6.0 && long_dist < 59.0)
          {
              // If in sweet spot, average the two
              if (short_dist > 6.0 && short_dist < 9.0)
              {
                  float ave = (short_dist + long_dist)/2.0;
                  comb_dist = ave;
                  return ave;
                  //lcd.print(ave);
              }
              // If the long is reading above 6 but the short is reading below, its too close for long
              else if (short_dist <=6.0)
              {
                  comb_dist = short_dist;
                  return short_dist;
                  //lcd.print(short_dist);
              }
              // Otherwise its too far for short
              else
              {
                  comb_dist = long_dist;
                  return long_dist;
                  //lcd.print(long_dist);
              }
          }
          // Too far for both
          else
          {
            //lcd.print("00.00");
            comb_dist = 60.0f;
            return 60.0f;
          }      
  }
  
  // Takes analogue distance and returns distance in inches
  float Long_Distance(int analogue)
  {
      return (3617.3 * pow(analogue, -0.889)) - 4.0;
  }
  
  // Takes analogue distance and returns distance in inches
  float Short_Distance(int analogue)
  {
      return (643.8 * pow(analogue, -0.895)) -0.5 ;
  }
  
  
  // Store all distance readings
  void Get_Distance_Readings()
  {
      if (Check_Timer(dist_timer) >= 10)
      {
          // Reset the timer
          Set_Timer(dist_timer);
         
          short_read[read_count]=analogRead(SFSensor);
          long_read[read_count]=analogRead(LFSensor);  
          right_read[read_count]=analogRead(SRSensor);  
          left_read[read_count]=analogRead(SLSensor);   
          
          read_count++;
          if (read_count >= 10)
          {
              
              float short_sum = 0;
              float long_sum = 0;
              float left_sum = 0;
              float right_sum = 0;
  
               // SORT
               int short_min=1000;
               int long_min = 1000;
               int left_min = 1000;
               int right_min = 1000;
                for (int i=0; i<10; i++) {
                     for (int j=1;j<10;j++){
                         if (short_read[j]<short_read[j-1]){
                             short_min=short_read[j-1];
                            short_read[j-1]=short_read[j];
                            short_read[j]=short_min;
                          }
                           if (long_read[j]<long_read[j-1]){
                             long_min=long_read[j-1];
                            long_read[j-1]=long_read[j];
                            long_read[j]=long_min;
                          }
                           if (left_read[j]<left_read[j-1]){
                             left_min=left_read[j-1];
                            left_read[j-1]=left_read[j];
                            left_read[j]=left_min;
                          }
                          if (right_read[j]<right_read[j-1]){
                             right_min=right_read[j-1];
                            right_read[j-1]=right_read[j];
                            right_read[j]=right_min;
                          }
                    }
                }
               
                // Average
              for (read_count = 2; read_count < 8; read_count++)
              {
                  short_sum += short_read[read_count];
                  long_sum += long_read[read_count];
                  left_sum += left_read[read_count];
                  right_sum += right_read[read_count];
              }
              read_count = 0;
              
              short_sum /= 6;
              long_sum /= 6;
              left_sum /= 6;
              right_sum /= 6;
              
              short_dist = Short_Distance(short_sum);
              long_dist = Long_Distance(long_sum);
              left_dist = Short_Distance(left_sum);
              right_dist = Short_Distance(right_sum);
              comb_dist = Front_Combined();
          }
      }
      
  }
  
  
  
  
  //****************************************************************************************
  // TIMERS
  //****************************************************************************************
  // To reset the timer call Set_Timer(). Then you can check elapsed_time to see how much time as passed since the timer was set
  void Update_Time()
  {
      current_time = millis(); 
  }
  
  unsigned int Check_Timer(unsigned long &timer)
  {
      return current_time - timer;
  }
  
  void Set_Timer(unsigned long &timer)
  {
      timer = current_time;
  }
  
  
  //****************************************************************************************
  // DISPLAY
  //****************************************************************************************
  void Display()
  {
     
      lcd.clear();
      
      int mazeComplete = 0;
      for (int i = 0; i < 16; i++)
      {
          lcd.setCursor(i,0);
          if (maze.IsVisited(i))
          {
              lcd.print("X");
              mazeComplete++;
          }
          else
              lcd.print("0");
      }
      // Navigated the whole maze
      if (mazeComplete == 16 && globalState != PATH)
      {
          globalState = MENU;
      }
      
      lcd.setCursor(0,1);
      lcd.print("G");
      lcd.setCursor(1,1);
      lcd.print(currentCell);

      lcd.setCursor(3, 1);
      lcd.print("W");
      lcd.setCursor(4, 1);
      if (wallW == YES)
          lcd.print("X");
      else if (wallW == NO)
          lcd.print("0");
      else
          lcd.print("U");

      lcd.setCursor(6, 1);
      lcd.print("N");
      lcd.setCursor(7, 1);
      if (wallN == YES)
          lcd.print("X");
      else if (wallN == NO)
          lcd.print("0");
      else
          lcd.print("U");
     
      lcd.setCursor(9, 1);
      lcd.print("E");
      lcd.setCursor(10, 1);
      if (wallE == YES)
          lcd.print("X");
      else if (wallE == NO)
          lcd.print("0");
      else
          lcd.print("U");

       lcd.setCursor(12, 1);
      lcd.print("S");
      lcd.setCursor(13, 1);
      if (wallS == YES)
          lcd.print("X");
      else if (wallS == NO)
          lcd.print("0");
      else
          lcd.print("U");
     
  }

