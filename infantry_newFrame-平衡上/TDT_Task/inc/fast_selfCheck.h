#ifndef FAST__SELFCHECK_H
#define FAST__SELFCHECK_H

#include "board.h"

class FastSelfCheck
{
public:
	void run();
	uint64_t lastKeySwitchTime;
	u8 selfCheckingFlag = 0;
};

extern FastSelfCheck fastSelfCheck;

#endif
