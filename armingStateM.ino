int armState = 0;

void armingStateM()
{
  if (!armed)
  {
    switch (armState)
    {
      case 0:
      case 2:
        if (unRollIn > 1900 && unPitchIn > 1900)
        {
          armState++;
        }
        break;

      case 1:
      case 3:
        if (unRollIn < 1100 && unPitchIn < 1100)
        {
          armState++;
        }
        break;
      case 4:
        if(unThrottleIn < 1100)
        {
          armed = true;
          armState = 0;
        }
        break;
    }
  }
  else
  {
    switch (armState)
    {
      case 0:
      case 2:
        if (unRollIn > 1900 && unPitchIn > 1900)
        {
          armState++;
        }
        break;

      case 1:
      case 3:
        if (unRollIn < 1100 && unPitchIn < 1100)
        {
          armState++;
        }
        break;
      case 4:
        armed = false;
        armState = 0;
        break;
    }
  }
}

