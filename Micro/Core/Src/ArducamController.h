/*
 * ArducamController.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Isayah
 */

#ifndef SRC_ARDUCAMCONTROLLER_H_
#define SRC_ARDUCAMCONTROLLER_H_

class ArducamController {
public:
	ArducamController();
	virtual ~ArducamController();
	void readFrameBuffer();
	void setSettings();
	void singleCapture();
};

#endif /* SRC_ARDUCAMCONTROLLER_H_ */
