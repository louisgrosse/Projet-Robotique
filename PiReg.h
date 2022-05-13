/*
 * PiReg.h
 *
 *  Created on: 5 May 2022
 *      Author: louis
 */

#ifndef PIREG_H_
#define PIREG_H_

int16_t Pi_Reg(float amplitude, float goal);
//start the PI regulator thread
void pi_regulator_start(void);


#endif /* PIREG_H_ */

