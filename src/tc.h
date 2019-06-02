void tcConfigure(int sampleRate);

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing();

//This function enables TC5 and waits for it to be ready
void tcStartCounter();

//Reset TC5 
void tcReset();

//disable TC5
void tcDisable();
