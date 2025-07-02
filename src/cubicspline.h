#ifndef CUBICSPLINE_H
#define CUBICSPLINE_H
struct CubicSpline {
    std::vector<double> x, a, b, c, d;  

    void setPoints(const std::vector<double>& xs,
                    const std::vector<double>& ys) {
        int n = xs.size();
        x = xs;
        a = ys;

        std::vector<double> h(n-1), alpha(n-1);
        for (int i = 0; i < n-1; ++i)
            h[i] = xs[i+1] - xs[i];

        for (int i = 1; i < n-1; ++i)
            alpha[i] = 3.0*(a[i+1] - a[i]) / h[i]
                     - 3.0*(a[i]   - a[i-1]) / h[i-1];

        c.assign(n, 0.0);
        std::vector<double> l(n), mu(n), z(n);
        l[0] = 1.0;  mu[0] = z[0] = 0.0;
        for (int i = 1; i < n-1; ++i) {
            l[i] = 2*(xs[i+1] - xs[i-1]) - h[i-1]*mu[i-1];
            mu[i] = h[i] / l[i];
            z[i]  = (alpha[i] - h[i-1]*z[i-1]) / l[i];
        }
        l[n-1] = 1.0;  z[n-1] = c[n-1] = 0.0;

        b.assign(n-1, 0.0);
        d.assign(n-1, 0.0);
        for (int j = n-2; j >= 0; --j) {
            c[j] = z[j] - mu[j]*c[j+1];
            double hi = h[j];
            b[j] = (a[j+1]-a[j])/hi - hi*(c[j+1]+2*c[j])/3.0;
            d[j] = (c[j+1] - c[j]) / (3.0*hi);
        }
    }

    double operator()(double xi) const {
        int i = std::upper_bound(x.begin(), x.end(), xi) - x.begin() - 1;
        if (i < 0)            i = 0;
        else if (i >= b.size()) i = b.size()-1;
        double dx = xi - x[i];
        return a[i]
             + b[i]*dx
             + c[i]*dx*dx
             + d[i]*dx*dx*dx;
    }

    double getDerivative(double xi) const {
        int i = std::upper_bound(x.begin(), x.end(), xi) - x.begin() - 1;
        if (i < 0)            i = 0;
        else if (i >= b.size()) i = b.size()-1;
        double dx = xi - x[i];
        return b[i]
             + 2.0*c[i]*dx
             + 3.0*d[i]*dx*dx;
    }
};

#endif //CUBICSPLINE_H
