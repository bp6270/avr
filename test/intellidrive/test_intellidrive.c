#include "CUnit/Basic.h"

int init_suite(void)  { return 0; }
int clean_suite(void) { return 0; }

int is_even (int x)
{
  return (x % 2 == 0);
}

void test_is_even (void)
{
  CU_ASSERT(is_even(1)  == 0);
  CU_ASSERT(is_even(2)  == 1);
  CU_ASSERT(is_even(3)  == 0);
}

int main (void)
{
   CU_pSuite pSuite = NULL;

   /* Initialize CUnit test registry */
   if (CUE_SUCCESS != CU_initialize_registry())
      return CU_get_error();

   /* Add suite to registry */
   pSuite = CU_add_suite("Basic_Test_Suite", init_suite, clean_suite);
   if (NULL == pSuite) {
      CU_cleanup_registry();
      return CU_get_error();
      }

   /* add test to suite */
   if ((NULL == CU_add_test(pSuite, "test_is_even", test_is_even)))
   {
      CU_cleanup_registry();
      return CU_get_error();
   }

   /* Run tests using Basic interface */
   CU_basic_run_tests();

   /* Clean up registry and return */
   CU_cleanup_registry();
   return CU_get_error();
}
