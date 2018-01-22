#include <string>                         // cout, stod
#include <iomanip>                        // Required for cout precision
#include <eigen3/Eigen/Dense>             // Required for matrix inverse calcs
#include <eigen3/Eigen/Geometry>          // Required for matrix math
#include <ros/ros.h>

using namespace std;

void print4x4Matrix (const Eigen::Matrix4d &matrix)
{
  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
  printf ("    |  0      0      0      1     | \n");
  printf ("\n");
}



void print3x1Vector (const Eigen::Vector3d &v)
{
  printf ("\n");
  printf ("    | %6.3f | \n", v(0));
  printf ("d = | %6.3f | \n", v(1));
  printf ("    | %6.3f | \n", v(2));
  printf ("\n");
}


void print3x3Matrix (const Eigen::Matrix3d &matrix)
{
  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("\n");
}



void printAffine (const Eigen::Affine3d &matrix)
{
  //printf ("Transformation matrix :\n");
  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
  printf ("T = | %6.3f %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
  printf ("    |  0      0      0      1     | \n");
  printf ("\n");
}



Eigen::Affine3d invertAffine (const Eigen::Affine3d &T)
{
  bool invertible;
  double determinant;
  Eigen::Matrix3d R_inverse;
  Eigen::Vector3d d;
  Eigen::Vector3d d_inverse;
  Eigen::Affine3d T_inverse = Eigen::Affine3d::Identity();

  // Check to make sure the rotation component R is invertible
  Eigen::Matrix3d R = T.rotation();
  R.computeInverseAndDetWithCheck(R_inverse, determinant, invertible);
  cout <<fixed<<setprecision(2)<<"Determinant: " << determinant << endl;

  printf("Matrix to be inverted:\n");            printAffine(T);
  printf("Rotation matrix:\n");                  print3x3Matrix(R);
  printf("Inverted rotation matrix (R^-1)\n");   print3x3Matrix(R_inverse);

    if (invertible)
    {
        d = T.translation();
        printf("Original vector (d)\n");         print3x1Vector(d);
        d_inverse = (-1.0) * R_inverse * d;
        printf("Inverted vector (-R^-1 * d)\n"); print3x1Vector(d_inverse);
        T_inverse.rotate(R_inverse);
        printf("Output after rotation\n");       printAffine(T_inverse);
        T_inverse.translate(d_inverse);          // this line is failing
        printf("Output after translation\n");    printAffine(T_inverse);
        T_inverse.translation() << d_inverse(0), d_inverse(1), d_inverse(2);
        printf("Translate using \"<<\"\n");      printAffine(T_inverse);
    }
}


int main()
{
  Eigen::Affine3d test_affine = Eigen::Affine3d::Identity();
  Eigen::Matrix4d test_matrix = Eigen::Matrix4d::Identity();
  test_matrix <<  0.401,  0.916, -0.003, -4995029.924,
                 -0.916,  0.401,  0.007, -1500043.320,
                  0.007,  0.000,  1.000, -5066.161,
                  0,      0,      0,      1;
  test_affine.rotate(test_matrix.block<3,3>(0,0));
  test_affine.translate(test_matrix.block<3,1>(0,3));
  invertAffine(test_affine);
  return 0;
}
