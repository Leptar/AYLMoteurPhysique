#include "Tests/3DVectorTest.h"

void Vector3DTest::TestAddition() {
	Vector3D v1(1, 2, 3);
	Vector3D v2(4, 5, 6);
	Vector3D v3 = v1 + v2;
	assert(v3.x == 5);
	assert(v3.y == 7);
	assert(v3.z == 9);

	std::cout << "Vector3D addition validé" << std::endl;
}

void Vector3DTest::TestSubtraction() {
	Vector3D v1(1, 2, 3);
	Vector3D v2(4, 5, 6);
	Vector3D v3 = v1 - v2;
	assert(v3.x == -3);
	assert(v3.y == -3);
	assert(v3.z == -3);

	std::cout << "Vector3D soustraction validé" << std::endl;
}

void Vector3DTest::TestScalarProduct() {
	Vector3D v1(1, 2, 3);
	Vector3D v2 = v1.scalar(2);
	assert(v2.x == 2);
	assert(v2.y == 4);
	assert(v2.z == 6);

	std::cout << "Vector3D produit scalaire validé" << std::endl;
}

void Vector3DTest::TestDot() {
	Vector3D v1(1, 0, 0);
	Vector3D v2(0, 1, 0);

	assert(v1.dot(v2) == 0);

	Vector3D v3(2, 3, 4);
	assert(v3.dot(v3) == v3.GetSquareNorm());

	std::cout << "Vector3D produit scalaire (dot) validé" << std::endl;
}

void Vector3DTest::TestCross() {
	Vector3D v1(1, 0, 0);
	Vector3D v2(0, 1, 0);
	Vector3D v3 = v1.cross(v2);

	assert(v3.x == 0);
	assert(v3.y == 0);
	assert(v3.z == 1);

	std::cout << "Vector3D produit vectoriel (cross) validé" << std::endl;
}

void Vector3DTest::TestNorms() {
	Vector3D v1(3, 4, 0);

	assert(v1.GetNorm() == 5.0f);
	assert(v1.GetSquareNorm() == 25.0f);

	std::cout << "Vector3D normes validées" << std::endl;
}

void Vector3DTest::TestNormalize() {
	Vector3D v1(3, 0, 0);
	Vector3D n = v1.normalize();

	assert(std::fabs(n.GetNorm() - 1.0f) < 1e-6);

	std::cout << "Vector3D normalisation validée" << std::endl;
}

void Vector3DTest::RunAllTests() {
	TestAddition();
	TestSubtraction();
	TestScalarProduct();
	TestDot();
	TestCross();
	TestNorms();
	TestNormalize();
	std::cout << "=== Tous les tests Vector3D validés 🎉 ===" << std::endl;
}
