uint16_t distanceFromTarget(uint16_t targetF, uint16_t currentF){
	if(((targetF+DISTANCE_THRESHOLD) >= currentF) && ((targetF-DISTANCE_THRESHOLD) <= currentF )){
		PORTB = 0;
		PORTB |= (1 << GREEN_LED);
	}
	else if(((targetF+DISTANCE_THRESHOLD*2) >= currentF) && (targetF-DISTANCE_THRESHOLD*2) <= currentF){
		PORTB = 0;
		PORTB |= (1 << ORANGE_TOP_LED);
		PORTB |= (1 << ORANGE_BOTTOM_LED);
	}
	else if((1023 >= currentF) && (0 <= currentF)){
		PORTB = 0;
		PORTB |= (1 << RED_TOP_LED);
		PORTB |= (1 << RED_BOTTOM_LED);
	}
}
