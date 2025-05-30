#include <iostream>
#include<matplot/matplot.h>


 int main()
 {
    using namespace matplot;
    auto x = linspace(0, 2 * pi, 100);
    auto y = transform(x, [](double x) { return sin(x); });
    plot(x, y);
    show();
    return 0;
 }