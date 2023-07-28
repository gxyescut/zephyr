#include "stdio.h"


int test_exception ()
{
  int y = 0;
  try {
      if( y == 0 )
      {
        throw -1;
      }
  }
  catch (...) {
    printf("caught an exception! \n");
  }

  return 0;
}

extern "C" {
int test_exception_demo ()
{
   test_exception();
   return 0;
}

}
