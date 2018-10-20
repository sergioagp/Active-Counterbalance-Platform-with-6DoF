#include "platform.h"

int main(void) {
	CPlatform* platform = CPlatform::getInstance();
	platform->start();
	
	return 0;
}
