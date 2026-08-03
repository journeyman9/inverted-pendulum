#include <iostream>
#include "kalman.h"

/*
Severity	Code	Description	Project	File	Line
Error		required from 'void std::_Destroy(_ForwardIterator, _ForwardIterator, std::allocator<_T2>&) [with _ForwardIterator = std::vector<float>*; _Tp = std::vector<float>]'	FREERTOS_SHELL	C:\Users\journ\Documents\GitHub\avr-libstdcpp\include\bits\alloc_traits.h	738

Severity	Code	Description	Project	File	Line
Error		required from 'void std::_Destroy(_ForwardIterator, _ForwardIterator) [with _ForwardIterator = std::vector<float>*]'	FREERTOS_SHELL	C:\Users\journ\Documents\GitHub\avr-libstdcpp\include\bits\stl_construct.h	185


Severity	Code	Description	Project	File	Line
Error		required from 'std::vector<_Tp, _Alloc>::~vector() [with _Tp = std::vector<float>; _Alloc = std::allocator<std::vector<float> >]'	FREERTOS_SHELL	C:\Users\journ\Documents\GitHub\avr-libstdcpp\include\bits\stl_vector.h	680

Severity	Code	Description	Project	File	Line
Error		required from 'static void std::_Destroy_aux<<anonymous> >::__destroy(_ForwardIterator, _ForwardIterator) [with _ForwardIterator = std::vector<float>*; bool <anonymous> = false]'	FREERTOS_SHELL	C:\Users\journ\Documents\GitHub\avr-libstdcpp\include\bits\stl_construct.h	152

Severity	Code	Description	Project	File	Line
Error		body of constexpr function 'constexpr _Tp* std::__addressof(_Tp&) [with _Tp = std::vector<float>]' not a return-statement	FREERTOS_SHELL	C:\Users\journ\Documents\GitHub\avr-libstdcpp\include\bits\move.h	50

Severity	Code	Description	Project	File	Line
Error		'__builtin_addressof' was not declared in this scope	FREERTOS_SHELL	C:\Users\journ\Documents\GitHub\avr-libstdcpp\include\bits\move.h	50


constexpr uint8_t N = 4;

  };

  struct Matrix {
      float data[N][N];
  };

#define STATES 4

  typedef float Vector[STATES];
  typedef float Matrix[STATES][STATES];

#define N 4

  typedef struct {
      float v[N];
  } Vector;

  typedef struct {
      float m[N][N];
  } Matrix;

*/

int main() {
    
    const std::array<float, 4> x0{{0.0, 0.0, 0.0, 0.0}};
    Kalman observer(x0);
    
    float u{0.01};
    observer.predict(u);

    std::array<float, 4> z{{0.215, 0.05, 0.9873, -0.05}};
    observer.update(z);
    
    for (int i=0; i<x0.size(); i++) {
        std::cout << "x_hat_" << i << ": " << observer.getStateEstimate()[i] << std::endl;
    }
    return 0;
}