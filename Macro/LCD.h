const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;

int direction = 0;
int autonVal = 0;

void waitForPress()
{
	while(nLCDButtons == 0){}
	wait1Msec(5);
}

void waitForRelease()
{
	while(nLCDButtons != 0){}
	wait1Msec(5);
}

task selectAuton()
{
	bLCDBacklight = true;

	int count = 0;

	clearLCDLine(0);
	clearLCDLine(1);

	while(nLCDButtons != leftButton || nLCDButtons != rightButton)
	{
		displayLCDCenteredString(0, "Choose a Side");
		displayLCDCenteredString(1, "Left       Right");
		waitForPress();
	}
	if(nLCDButtons == leftButton)
	{
		waitForRelease();
		direction = 1;
	}
	else if(nLCDButtons == rightButton)
	{
		waitForRelease();
		direction = 2;
	}

	while(nLCDButtons != centerButton)
	{

		switch(count){
		case 0:

			displayLCDCenteredString(0, "Center Collect");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count = 3;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 1:

			displayLCDCenteredString(0, "Knock Stars");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 2:

			displayLCDCenteredString(0, "Autonomous 3");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count++;
			}
			break;
		case 3:

			displayLCDCenteredString(0, "Autonomous 4");
			displayLCDCenteredString(1, "<		 Enter		>");
			waitForPress();

			if(nLCDButtons == leftButton)
			{
				waitForRelease();
				count--;
			}
			else if(nLCDButtons == rightButton)
			{
				waitForRelease();
				count = 0;
			}
			break;
		default:
			count = 0;
			break;
		}
	}
	clearLCDLine(0);
	clearLCDLine(1);
	switch(count){
	case 0:
		autonVal = 1;
		break;
	case 1:
		autonVal = 2;
		break;
	case 2:
		autonVal = 3;
		break;
	case 3:
		autonVal = 4;
		break;
	default:
		autonVal = 0;
		break;
	}
}
