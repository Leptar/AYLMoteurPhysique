#pragma once

#include "3DVector.h"

#include <assert.h>
#include <iostream>


class Vector3DTest {
public:
	void TestAddition();
	void TestSubtraction();
	void TestScalarProduct();
	void TestDot();
	void TestCross();
	void TestNormalize();
	void TestNorms();

	void RunAllTests();



};
