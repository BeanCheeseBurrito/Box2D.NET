// Temporary fix for undefined symbol errors when statically linking on windows.
int __isnanf(float) { }
int __fpclassifyf(float) { }
int printf(const char *, ...) { }
