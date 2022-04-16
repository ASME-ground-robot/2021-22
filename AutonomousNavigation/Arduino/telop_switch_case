void setup() 
{

int DZL = 0.3 //Dead Zone Length 
              //(cause the dead zone is a square)

char Direction = "A"
int X = 0;
int Y = 0;


}

void loop() 
{


switch (Direction) 
{
  case A:
    if (inputY > DZL)                   // if loop for converting input to unit step
    {
      Y=1;
    else if (inputY < -DZL)
    {
      Y=-1;
    }
    else if (inputY > -DZL && inputY < DZL)
    {
      Y=0;
    }
    else if (inputX > DZL)
    {
      X=1;
    }
    else if (inputX < -DZL)
    {
      X=-1;
    }
    else if (inputX > -DZL && inputX < DZL)
    {
      X=0;
    }
    }                                   // end if loop for converting input to unit step

    switch (Yinput)                     // switch for converting input-unit step to Direction
    {

    case 1:
      switch (Xinput) 
      {
        case 0:
          Direction = "F"
          break;
        case 1:
          Direction = "FR"
          break;
        case -1:
          Direction = "FL"
          break;
      }
      break;

    case 0:
      switch (Xinput) 
      {
        case 0:
          Direction = "0"
          break;
        case 1:
          Direction = "R"
          break;
        case -1:
          Direction = "L"
          break;
      }
      break;

    case -1:
      switch (Xinput) 
      {
        case 0:
          Direction = "B"
          break;
        case 1:
          Direction = "BR"
          break;
        case -1:
          Direction = "BL"
          break;
      }
      break;
    }                                   // end of switch for converting input-unit step to Direction



  case F:                               //Forward code

    if (Y==1 && X==0) 
    {
      while (Y==1 && X==0)
      {
        //speed up to whatever
        if (Y==1 && X==0) 
        {
          while (Y==1 && X==0)
          {
          //maintain speed
          }
        }
      }
    else if (Y !== 1 || X !== 0)
    {
      //speed to zero
    }
    }
    Direction = 'A';
    break;

  case B:                               //Backward code

    if (Y==-1 && X==0) 
    {
      while (Y==-1 && X==0)
      {
        //speed up to whatever
        if (Y==-1 && X==0) 
        {
          while (Y==-1 && X==0)
          {
          //maintain speed
          }
        }
      }
    else if (Y !== -1 || X !== 0)
    {
      //speed to zero
    }
    }
    Direction = 'A';
    break;

  case L:                               //Left code

    if (Y==0 && X==-1) 
    {
      while (Y==0 && X==-1)
      {
        //speed up to whatever
        if (Y==0 && X==-1) 
        {
          while (Y==0 && X==-1)
          {
          //maintain speed
          }
        }
      }
    else if (Y !== 0 || X !== -1)
    {
      //speed to zero
    }
    }
    Direction = 'A';
    break;

  case R:                                //Right code

    if (Y==0 && X==1) 
    {
      while (Y==0 && X==1)
      {
        //speed up to whatever
        if (Y==0 && X==1) 
        {
          while (Y==0 && X==1)
          {
          //maintain speed
          }
        }
      }
    else if (Y !== 0 || X !== 1)
    {
      //speed to zero
    }
    }
    Direction = 'A';
    break;

  case FL:
    //If loop skid turn forward left
    Direction = 'A';
    break;

  case FR:
    //If loop skid turn forward right
    Direction = 'A';
    break;

  case BL:
    //If loop skid turn backward left
    Direction = 'A';
    break;

  case BR:
    //If loop skid turn backward right
    Direction = 'A';
    break;

  default:
    //motors(0);
    Direction = 'A';
    //Can be optional, but do WE need this?
    break;



}//end switch main



}//the end
