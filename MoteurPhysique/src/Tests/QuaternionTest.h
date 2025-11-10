#pragma once

class QuaternionTest {
public:
	static void RunAllTests();

private:
	static void Test_Default_Constructor();

	static void Test_Constructor();

	static void Test_CreateFromAxisAngle();

	static void Test_CreateFromEulerXYZ();

	static void Test_Identity();

	static void Test_Length();

	static void Test_Normalize();

	static void Test_Normalized();

	static void Test_Conjugate();

	static void Test_Inverse();

	static void Test_Multiply();

	static void Test_Rotate();

	static void Test_Slerp();
};
