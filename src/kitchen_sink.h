#if 0

int checkin (Point P){
    double A = majorAxis;
    double B = minorAxis;
    return (((P.x * P.x) / (A * A) + (P.y * P.y) / (B*B))  <= 1);
}

vector<double> get (vector<double> a, vector<double> b)
{
    for (int i = 0;i < a.size(); i ++)
        a[i] -= b[i];
    return a;
}

vector<vector<double> > prod (vector<vector<double> > A, vector<vector<double> > B)
{
    vector<vector<double> > product;
    
    int m = A.size();
    int n = A[0].size();
    
    for (int i = 0;i < m; i ++)
    {
        vector<double> row;
        for (int j = 0; j < B[0].size(); j ++)
        {
            double val = 0;
            for (int k = 0 ; k < n; k++)
                val += A[i][k] * B[k][j];
            row.push_back (val);
        }
        product.push_back (row);
    }
    
    return product;
}

vector<double> prod (vector<vector<double> > A, vector<double> x)
{
    vector<double> product;
    
    for (int i = 0;i < A.size(); i ++)
    {
        double val = 0.0;
        for (int j = 0;j < A[i].size() ; j ++)
            val += A[i][j] * x[j];
        product.push_back (val);
    }
    
    return product;
}

vector<double> prod (vector<double> A, vector<double> x)
{
    vector<double> product;
    
    for (int i = 0;i < A.size(); i ++)
        A[i] = A[i] * x[0];
    
    product = A;
    
    return product;
}


double dot_prod (vector<double> A, vector<double> x)
{
    double total = 0;
    
    for (int i = 0;i < A.size(); i ++)
        total += A[i] * x[i];
        
    return total;
}

vector<vector<double> > transpose (vector<vector<double> > A)
{
    vector<vector<double> > matrix (A[0].size(), vector<double> (A.size(),0)) ;
    for (int i = 0;i < A.size(); i ++)
    {
        for (int j = 0;j < A[0].size(); j ++)
            matrix[j][i] = A[i][j];
    }
    return matrix;
}

vector<vector<double> > transpose (vector<double> x)
{
    vector<vector<double> > matrix ;
    matrix.push_back (x);
    return matrix;
}

double inner_product(vector<double> vec)
{
    return prod (transpose(vec), vec)[0];
}

vector<double> get_gradient (vector<vector<double> > A, vector<double> x, vector<double> b)
{
    vector<double> gradient;
    int m = A.size();
    int n = x.size();
    
    for (int i = 0 ; i < n; i++)
    {
        vector<double> row;
        for (int j  = 0; j < m; j ++)
            row.push_back (A[j][i]);
        vector<double> temp;
        temp = get (prod(A,x),b);
        double val = 2 * dot_prod (row, temp);
        gradient.push_back (val);
    }
    
    return gradient ;
}

vector<double> gradient_descent_convex(int id, vector<vector<double> > A, vector<double> b, vector<Point> points, vector<double> fval, vector<double> last_gradient)
{
    int iter = 0;
    vector<double> x, y, z;
    vector<vector<double> > X;
    
    z = vector<double> (A[0].size(), 0);

    vector<double> eta;
    eta.push_back (PI/1000);
    double val ;
  //  x.push_back (-2*(points[1].x-gaussianCenter[id].x) * fval[1]);
  //  x.push_back (-2*(points[1].y-gaussianCenter[id].y) * fval[1]);
   // double norm = sqrt (x[0] * x[0] + x[1]*x[1]);
   // x[0]/=norm; x[1]/=norm;
    
 //   return x;/*
    x = last_gradient;
  
    while (iter < 1000)
    {
        y = get (x, prod (get_gradient(A,x,b), eta));
        x = y;
        X.push_back (x);
        iter++;
    }
    
    for (int i = 0;i < X.size(); i ++)
        z = get (z,X[i]);
    
    vector<double> temp;
    temp.push_back (-1.0/X.size());
    z = prod (z, temp);
    
    vector<double> err;
    
    err = get(b, prod (A,z));
    
 //   cout<<"Error is "<<inner_product (err)<<endl;
    
    return z;
}

/*
double gaussian (Point input, int gaussianId)
{
    double X = input.x - gaussianCenter[gaussianId].x;
    double Y = input.y - gaussianCenter[gaussianId].y;
    double varX = gaussianVar[gaussianId].x;
    double varY = gaussianVar[gaussianId].y;
    return exp ( - (X * X / (2*varX)) - (Y * Y/(2*varY)));
}

vector<double> getGaussian (vector<Point> points)
{
    vector<double> fval;
    
    for (int i = 0; i < points.size(); i ++)
    {
        double sum = 0;
        for (int j = 0; j < gaussianCenter.size(); j++)
            sum = sum + gaussian (points[i], j);
        fval.push_back (sum);
    }
    
    return fval;
}

double getGaussian (Point point)
{
    double fval;
    double sum = 0;
    for (int j = 0; j < gaussianCenter.size(); j++){
        sum = sum + gaussian (point, j);
    } 
    fval = sum;
    return fval;
}
*/

vector<Point> removeDuplicatePoints(vector<Point>& points, double eps = 1e-9) {
    vector<Point> cleaned;
    if (points.empty()) return cleaned;

    cleaned.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i) {
        if (!points[i].isNear(cleaned.back(), eps)) {
            cleaned.push_back(points[i]);
        }
    }
    return cleaned;
}


// This uses marching squares to get the contours/level sets
// https://en.wikipedia.org/wiki/Marching_squares
// https://www.baeldung.com/cs/marching-squares
vector<Point> getGaussianContours(double level,
                                  double stepSize,
                                  double xMin,
                                  double xMax,
                                  double yMin,
                                  double yMax) {

    vector<Point> levelSetPoints;

    for (double x = xMin; x < xMax; x += stepSize) {
        for (double y = yMin; y < yMax; y += stepSize) {
            // Grid corners
            Point p1(x, y);
            Point p2(x + stepSize, y);
            Point p3(x, y + stepSize);
            Point p4(x + stepSize, y + stepSize);

            double f1 = getGaussian(p1);
            double f2 = getGaussian(p2);
            double f3 = getGaussian(p3);
            double f4 = getGaussian(p4);

            // Check if the level set crosses the grid cell
            if ((f1 > level && f2 > level && f3 > level && f4 > level) ||
                (f1 < level && f2 < level && f3 < level && f4 < level)) {
                continue;
            }

            // Build the case index (bits: p1, p2, p4, p3)
            int idx = 0;
            if (f1 > level) idx |= 1;
            if (f2 > level) idx |= 2;
            if (f4 > level) idx |= 4;
            if (f3 > level) idx |= 8;

            // Interpolation lambda
            auto interp = [&](const Point& a, const Point& b, double fa, double fb) -> Point {
                double t = (level - fa) / (fb - fa);
                return Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
            };

            switch (idx) {
                case 0:
                case 15:
                    break;

                case 1:
                case 14:
                    levelSetPoints.push_back(interp(p1, p2, f1, f2));
                    levelSetPoints.push_back(interp(p1, p3, f1, f3));
                    break;

                case 2:
                case 13:
                    levelSetPoints.push_back(interp(p2, p1, f2, f1));
                    levelSetPoints.push_back(interp(p2, p4, f2, f4));
                    break;

                case 3:
                case 12:
                    levelSetPoints.push_back(interp(p1, p3, f1, f3));
                    levelSetPoints.push_back(interp(p2, p4, f2, f4));
                    break;

                case 4:
                case 11:
                    levelSetPoints.push_back(interp(p4, p2, f4, f2));
                    levelSetPoints.push_back(interp(p4, p3, f4, f3));
                    break;

                case 5: {
                    levelSetPoints.push_back(interp(p1, p2, f1, f2));
                    levelSetPoints.push_back(interp(p1, p3, f1, f3));
                    levelSetPoints.push_back(interp(p4, p2, f4, f2));
                    levelSetPoints.push_back(interp(p4, p3, f4, f3));
                    break;
                }

                case 6:
                case 9:
                    levelSetPoints.push_back(interp(p2, p1, f2, f1));
                    levelSetPoints.push_back(interp(p4, p3, f4, f3));
                    break;

                case 7:
                case 8:
                    levelSetPoints.push_back(interp(p1, p3, f1, f3));
                    levelSetPoints.push_back(interp(p4, p3, f4, f3));
                    break;

                case 10: {
                    levelSetPoints.push_back(interp(p1, p3, f1, f3));
                    levelSetPoints.push_back(interp(p2, p4, f2, f4));
                    levelSetPoints.push_back(interp(p1, p2, f1, f2));
                    levelSetPoints.push_back(interp(p3, p4, f3, f4));
                    break;
                }
            }
        }
    }


    vector<Point> cleaned = removeDuplicatePoints(levelSetPoints);

    if (cleaned.size() < 2) {
        cout << "few level set points: " << cleaned.size() << endl;
        return cleaned;
    }


    return cleaned;
}


vector<double> get_Gaussian_vector (vector<Point> points, int id)
{
    vector<double> x;
    vector<double> fval = getGaussian (points);
    //below is buggy, does not incorporate variance
    x.push_back (-2*(points[1].x-gaussianCenter[id].x) * fval[1]);
    x.push_back (-2*(points[1].y-gaussianCenter[id].y) * fval[1]);
    return x;
}


double gradient_modulo (double gradient)
{
    if (gradient > PI)
        gradient = gradient - 2*PI;
    else if (gradient < -PI)
        gradient = gradient + 2*PI;
    return gradient;
}

vector<vector<double> > inverse (vector<vector<double> > A)
{
    double a , b , c , d;
    a = A[0][0] ;
    b = A[0][1] ;
    c = A[1][0] ;
    d = A[1][1] ;
    
    double det = a*d - b*c;
    
    if (abs(det) < 1e-9)
    {
        cout <<"Exception non-invertible!"<<endl;
        cout << a << " " << b << endl;
        cout << c << " " << d << endl;
        exit(0);
    }
    
    A[0][0] = d/det;
    A[0][1] = -b/det;
    A[1][0] = -c/det;
    A[1][1] = a/det;
    
    return A ;
}


vector<double> solve (vector<double> equation)
{   
    vector<double> solutions;
    double discriminant = 0;
    double a = equation[0];
    double b = equation[1];
    double c = equation[2];
    
    discriminant = b*b - 4*a*c;
   
    // getting negatives discriminants that should be considered zero
    // if considered zero, we have one solution 
    if (abs(discriminant)  < 1e-9){ 
        solutions.push_back ((-b) / (2*a));
        solutions.push_back ((-b) / (2*a));
        return solutions;
    }else if (discriminant < 0){
        return solutions;
    }else{
        solutions.push_back ( (sqrt (discriminant) - b) / (2*a));
        solutions.push_back ( (-sqrt (discriminant) - b) / (2*a));
        
        return solutions;
    }
    
}

vector<double> polynomial_product(const vector<double>& p1, const vector<double>& p2) {
    int degree1 = p1.size();
    int degree2 = p2.size();
    vector<double> result(degree1 + degree2 - 1, 0);

    for (int i = 0; i < degree1; ++i) {
        for (int j = 0; j < degree2; ++j) {
            result[i + j] += p1[i] * p2[j];
        }
    }

    return result;
}

double determinant(const std::vector<std::vector<double>>& M) {
    int n = M.size();

    if (n == 2) {
        // 2x2 determinant: ad - bc
        return M[0][0] * M[1][1] - M[0][1] * M[1][0];
    } else if (n == 3) {
        // 3x3 determinant using cofactor expansion
        return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
             - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
             + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    } else {
        std::cerr << "Only 2x2 and 3x3 matrices are supported.\n";
        exit(1);
    }
}

vector<double> getnullspace(vector<vector<double>> A) {
    int rows = A.size();
    int cols = A[0].size();

    if (rows != cols || (rows != 2 && rows != 3)) {
        cerr << "Only 2x2 or 3x3 matrices are supported!" << endl;
        exit(1);
    }

    vector<double> nullspace;

    if (rows == 2) {
        double a1 = A[0][0];
        double b1 = A[0][1];
        double a2 = A[1][0];
        double b2 = A[1][1];

        double x = 0, y = 0;
        if (abs(b1) > 1e-9) {
            y = -a1 / b1;
            x = 1;
        } else if (abs(b2) > 1e-9) {
            y = -a2 / b2;
            x = 1;
        }

        double norm = sqrt(x * x + y * y);
        x /= norm;
        y /= norm;

        nullspace.push_back(x);
        nullspace.push_back(y);
    } else if (rows == 3) {
        Eigen::Matrix<double, 3, 3> M;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                M(i, j) = A[i][j];
            }
        }

        Eigen::FullPivLU<Eigen::Matrix<double, 3, 3>> lu(M);
        Eigen::Matrix<double, 3, 1> kernel = lu.kernel().col(0);

    }

    return nullspace;
}


vector<vector<double> > get_pseudo_inv (vector<vector<double> > A)
{
    vector<vector<double> > pseudo_inv = A;

    A = prod (transpose(A),A);
    /*
     Singular Value Decomposition
     */
    
    vector<double> equation;
    std::vector<double> eigenvalue;
    if (A.size() == 2) {
        equation = polynomial_product({-1, A[0][0]}, {-1, A[1][1]});
        equation[2] -= (A[1][0] * A[0][1]);
        eigenvalue = solve(equation);
    } else if (A.size() == 3) {
        Eigen::Matrix<double, 3, 3> M;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                M(i, j) = A[i][j];
            }
        }
        Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> solver(M);
        Eigen::VectorXcd ev = solver.eigenvalues();
        // Convert complex eigenvalues to a vector of doubles
        for (int i = 0; i < ev.size(); ++i) {
            if (ev[i].imag() < 1e-9) {
            // If the eigenvalue is real, store the real part
            eigenvalue.push_back(ev[i].real());
            } else {
            // If the eigenvalue is complex, store the magnitude
            eigenvalue.push_back(std::abs(ev[i]));
            }
        }
    }
    else
    {   
        cout << "Exception! Not 2x2 or 3x3 matrix!"<<endl;
        exit(0);
    }

    
    vector<vector<double> > V, U, sigma;

    for (int i = 0; i < eigenvalue.size(); i ++){
        double egval = eigenvalue[i];
        
        
        vector<vector<double> > tonullspace = A;
        for(int j = 0; j < A.size(); j ++)
        {
            tonullspace[j][j] -= egval;
        }


        vector<double> eigenvector = getnullspace(tonullspace);

        // If nullspace fails (zero vector, all zeros, or NaNs), use a fallback
            bool invalid = eigenvector.empty() || std::all_of(eigenvector.begin(), eigenvector.end(), [](double v) {
                return v == 0.0 || std::isnan(v) || std::isinf(v);
            });

        if (invalid) {
            eigenvector = std::vector<double>(A.size(), 0.0);
            eigenvector[i] = 1.0; // fallback: unit vector in ith direction
        }

        // Normalize
        double norm = std::sqrt(std::inner_product(eigenvector.begin(), eigenvector.end(), eigenvector.begin(), 0.0));
        for (double& val : eigenvector) val /= norm;
    
        V.push_back (eigenvector);
        vector<double> temp (A.size(),0);
        if (eigenvalue[i]>=0)
            temp[i] = sqrt (eigenvalue[i]);
        else if (abs(eigenvalue[i]) < 1e-9)
            temp[i] = 0;
        else
        {
            cout<<"Negative eigenvalue! "<<eigenvalue[i]<< endl;
            exit(0);
        }
        sigma.push_back (temp);
    }
    
    U = pseudo_inv;
    U = prod (U, V);
    U = prod (U, sigma);
    
    for (int i = 0; i < 2; i ++){
        if (abs(sigma[i][i])>0){
            sigma[i][i] = 1/sigma[i][i] + 1e-9;
        }
    }
        
    pseudo_inv = V;
    pseudo_inv = prod (pseudo_inv, sigma);
    pseudo_inv = prod (pseudo_inv, transpose(U));
    
    return pseudo_inv;
}

vector<double> gradient_matrix_solver (vector<vector<double> > A, vector<double> b)
{
    
    vector<vector<double> > inv;
  //  inv = get_pseudo_inv (prod (transpose(A),A));
    inv = get_pseudo_inv (A);
  /*  cout<<"A below"<<endl;
    cout<<A[0][0]<<" "<<A[0][1]<<endl;
    cout<<A[1][0]<<" "<<A[1][1]<<endl;
    cout<<"Pseudo inverse below. "<<endl;
    cout<<inv[0][0]<<" "<<inv[0][1]<<endl;
    cout<<inv[1][0]<<" "<<inv[1][1]<<endl;
   */
 //   inv = prod (inv, transpose(A));
    vector<double> gradient = prod (inv, b);
  //  cout<<gradient[0]<<" gr "<<gradient[1]<<endl;
    return gradient;
}

vector<double> gradient_LSQ (vector<Point> points)
{
    vector<double> fval = getGaussian (points);
    vector<vector<double> > A;
    vector<double> b;
    
    b.push_back (fval[0] - fval[1]);
    b.push_back (fval[2] - fval[1]);
    
    vector<double> row;
    row.push_back (points[0].x - points[1].x);
    row.push_back (points[0].y - points[1].y);
    
    double gr[2]={0,1};
    
    row[1] /= row[0] + 1e-9;
    b[0] /= row[0] + 1e-9;
    row[0] = 1;
    A.push_back (row);

    
    row.erase (row.begin(), row.end());
    row.push_back (points[2].x - points[1].x);
    row.push_back (points[2].y - points[1].y);
    
    row[1] /= row[0] + 1e-9;
    b[1] /= row[0] + 1e-9;
    row[0] = 1;
    A.push_back (row);
    
    return gradient_matrix_solver (A,b);
}


// This function computes the gradient of the curvature at a point.
// This was originally supposed to be used in finding the gradient
// of the critical path.
std::vector<double> curvature_gradient_LSQ(
    std::map<Point, double> surroundingCurvatures,
    std::pair<Point, double> crossingPointCurvature) {

    // Ensure there are at least 3 points (including the point to solve)
    if (surroundingCurvatures.size() < 2) {
        std::cerr << "Insufficient points for gradient calculation." << std::endl;
        return {};
    }

    // Extract the coordinates and Gaussian values for the surrounding points
    Point pointToSolve = crossingPointCurvature.first;
    double xB = pointToSolve.x;
    double yB = pointToSolve.y;
    double kb = crossingPointCurvature.second;

    std::vector<std::vector<double>> A;
    std::vector<double> b;

    for (auto& pointInfo : surroundingCurvatures) {
        Point point = pointInfo.first;
        double x = point.x;
        double y = point.y;
        double k = pointInfo.second;

        // Build the rows of matrix A and vector b
        A.push_back({x - xB, y - yB});
        b.push_back(k - kb);
    }
    // Compute the pseudo-inverse of A
    std::vector<std::vector<double>> A_pinv = get_pseudo_inv(A);

    // Solve for x = A_pinv * b
    std::vector<double> gradient = prod(A_pinv, b); 

    return gradient;
}


std::vector<double> concentration_gradient_LSQ(
    std::vector<Point> surroundingPoints,
    Point& pointToSolve) {

    // Ensure there are at least 3 points (including the point to solve)
    if (surroundingPoints.size() < 2) {
        std::cerr << "Insufficient points for gradient calculation." << std::endl;
        return {};
    }

    // Extract the coordinates and Gaussian values for the surrounding points
    double xB = pointToSolve.x;
    double yB = pointToSolve.y;
    double fb = getGaussian(pointToSolve);
    // cout << "concentration at pts: " << fb << endl;

    std::vector<std::vector<double>> A;
    std::vector<double> b;

    for (auto& point : surroundingPoints) {
        double x = point.x;
        double y = point.y;
        double f = getGaussian(point);
        // cout << "concentration at surps " << f << endl;

        // Build the rows of matrix A and vector b
        A.push_back({x - xB, y - yB});
        b.push_back(f - fb);
    }
    // Compute the pseudo-inverse of A
    std::vector<std::vector<double>> A_pinv = get_pseudo_inv(A);

    // Solve for x = A_pinv * b
    std::vector<double> gradient = prod(A_pinv, b); 

    return gradient;
}

// Function to compute the Hessian matrix
// more details about this in the Contour Estimation with Drones
// overleaf document
std::vector<std::vector<double>> hessian_LSQ(
    std::vector<Point> surroundingPoints,
    Point& pointToSolve,
    std::vector<double>& gradientAtPoint) {

    double xE = pointToSolve.x;
    double yE = pointToSolve.y;
    double fxe = gradientAtPoint[0];
    double fye = gradientAtPoint[1];
    double fe = getGaussian(pointToSolve);

    if (surroundingPoints.size() < 3) {
        std::cerr << "Insufficient points for gradient calculation." << std::endl;
        return {};
    }

    int numPoints = surroundingPoints.size();
    std::vector<std::vector<double>> A(numPoints, std::vector<double>(3, 0.0));
    std::vector<double> b(numPoints, 0.0);

    for (size_t i = 0; i < surroundingPoints.size(); ++i) {
        double x = surroundingPoints[i].x;
        double y = surroundingPoints[i].y;
        double f = getGaussian(surroundingPoints[i]); 

        A[i][0] = 0.5 * std::pow(x - xE, 2); 
        A[i][1] = (x - xE) * (y - yE);
        A[i][2] = 0.5 * std::pow(y - yE, 2);
        b[i] = f - fe - fxe * (x - xE) - fye * (y - yE);
    }

    std::vector<std::vector<double>> A_pinv = get_pseudo_inv(A); 

    // Solve for x = A_pinv * b
    if (A_pinv[0].size() != b.size()) {
        cerr << "Matrix dimensions do not match for multiplication!" << endl;
        cerr << "A_pinv size: " << A_pinv[0].size() << ", b size: " << b.size() << endl;
        exit(1);
    }
    std::vector<double> x = prod(A_pinv, b);

    // Construct the Hessian matrix
    std::vector<std::vector<double>> H(2, std::vector<double>(2, 0.0));
    H[0][0] = x[0];
    H[0][1] = x[1];
    H[1][0] = x[1];
    H[1][1] = x[2];

    return H;
}

// Function to calculate curvature from Hessian and gradient
double calc_curvature(std::vector<double>& g, 
    std::vector<std::vector<double>>& H) {
    double Fx = g[0];
    double Fy = g[1];

    double Fxx = H[0][0];
    double Fxy = H[0][1];
    double Fyy = H[1][1];

    double num = -Fy * Fy * Fxx + 2 * Fx * Fy * Fxy - Fx * Fx * Fyy;

    double denom = std::pow(Fx * Fx + Fy * Fy, 1.5);

    return num / denom;
}

// The calc_curvature_LSQ function is a wrapper that calculates 
// the curvature at a point using LSQ.
double calc_curvature_LSQ(
    std::vector<Point> surroundingPoints,
    Point& pointToSolve)
{
    // Calculate the gradient at pointB using three points.
    std::vector<double> gradB = concentration_gradient_LSQ(surroundingPoints, pointToSolve); 

 
    // Calculate the Hessian using nine points, the computed gradient, and the concentration function.
    std::vector<std::vector<double>> H = hessian_LSQ(surroundingPoints, 
                                                     pointToSolve, gradB);

 
    // Calculate the curvature based on the gradient and Hessian.
    double curvature = calc_curvature(gradB, H);
 
    // Return the computed gradient and curvature.
    return curvature;
}


// Helper function to compute numerical gradient
std::vector<double> compute_gradient(const std::vector<double>& values) {
    size_t n = values.size();
    std::vector<double> gradient(n, 0.0);

    if (n < 2) {
        std::cerr << "Insufficient data points for gradient calculation." << std::endl;
        return gradient;
    }

    // Central difference for interior points
    for (size_t i = 1; i < n - 1; ++i) {
        gradient[i] = (values[i + 1] - values[i - 1]) / 2.0;
    }

    // Forward difference for the first point
    gradient[0] = values[1] - values[0];

    // Backward difference for the last point
    gradient[n - 1] = values[n - 1] - values[n - 2];

    return gradient;
}

// Function to compute the difference between consecutive elements
std::vector<double> compute_diff(const std::vector<double>& values) {
    size_t n = values.size();
    std::vector<double> diff(n - 1, 0.0);

    for (size_t i = 0; i < n - 1; ++i) {
        diff[i] = values[i + 1] - values[i];
    }

    return diff;
}

// Function to compute curvature derivatives using central differences
// helper function compute_graddient
std::vector<double> compute_curvature_derivative(const std::vector<Point>& path,
                                                 const std::vector<double>& k) {

    // Extract x and y coordinates from the path
    size_t n = path.size();
    std::vector<double> x(n), y(n);
    for (size_t i = 0; i < n; ++i) {
        x[i] = path[i].x;
        y[i] = path[i].y;
    }

    // Compute gradients
    std::vector<double> dx = compute_gradient(x);
    std::vector<double> dy = compute_gradient(y);
    std::vector<double> dk = compute_gradient(k);
    

    // Compute ds (arc length differences)
    std::vector<double> diff_x = compute_diff(x);
    std::vector<double> diff_y = compute_diff(y);
    n = diff_x.size();
    std::vector<double> ds(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
        ds[i] = std::sqrt(diff_x[i] * diff_x[i] + diff_y[i] * diff_y[i]);
    }

    // Compute ds_mid (midpoints of ds)
    std::vector<double> ds_mid(ds.size() + 1, 0.0);
    ds_mid[0] = ds[0];
    for (size_t i = 1; i < ds.size(); ++i) {
        ds_mid[i] = (ds[i - 1] + ds[i]) / 2.0;
    }
    ds_mid[ds.size()] = ds[ds.size() - 1];

    // Compute dk_ds
    std::vector<double> dk_ds(k.size(), 0.0);
    for (size_t i = 0; i < k.size(); ++i) {
        dk_ds[i] = dk[i] / ds_mid[i];
    }

    // Compute moving average of curvature derivatives
    size_t windowSize = 3; // Define the window size for the moving average
    std::vector<double> smoothedCurvatureDerivatives(k.size(), 0.0);

    for (size_t i = 0; i < k.size(); ++i) {
        double sum = 0.0;
        size_t count = 0;

        for (size_t j = i; j < std::min(i + windowSize, k.size()); ++j) {
            sum += dk_ds[j];
            ++count;
        }

        smoothedCurvatureDerivatives[i] = sum / count;
    }

    dk_ds = smoothedCurvatureDerivatives;

    return dk_ds;
}



double getAngle (vector<double> A)
{
    double normA = sqrt (A[0] * A[0] + A[1] * A[1]);
    double angle = acos (abs(A[0])/normA);
    if (A[0] >= 0 && A[1] >= 0)
        return angle;
    if (A[0] < 0 && A[1] >= 0)
        return PI - angle;
    if (A[0] < 0 && A[1] < 0)
        return PI + angle;
    return -angle;
}


Point::Point(double x, double y) : x(x), y(y) {}

Point Point::operator+(Point &first) {
    return Point(x + first.x, y + first.y);
}

Point Point::operator+=(Point &first){
    this->x += first.x;
    this->y += first.y;
    return *this;
}

Point Point::operator-(Point &first) {
    return Point(x - first.x, y - first.y);
}

Point Point::operator-=(Point &first){
    this->x -= first.x;
    this->y -= first.y;
    return *this;
}

Point Point::operator-() {
    return Point(-this->x, -this->y);
}

double Point::length() {
    return sqrt(x * x + y * y);
}

double Point::getX() const {
    return x;
}

double Point::getY() const {
    return y;
}

double PointUtil::orientation(Point& one, Point& two, Point& three) {
    double k=(two.getY() - one.getY())*(three.getX() - two.getX())-(two.getX() - one.getX()) * (three.getY() - two.getY());

    if(k>0) {
        return CLOCKWISE;
    } else {

        return COUNTERCLOCKWISE;
    }
}

Point PointUtil::vector(double angle, double length) {
    return Point(length * cos(angle), length * sin(angle));
}

Point PointUtil::perpendicular(Point &one, Point &two, double length, int orientation) {
    double delta_x = two.getX() - one.getX();
    double delta_y = two.getY() - one.getY();
    double angle = atan2(delta_y, delta_x);
    return vector(angle + (orientation * M_PI_2), length);
}


Line::Line(double m, double b) : m(m), b(b) {}

Line Line::buildByPoints(Point &start, Point &end) {
    double m = (end.getY() - start.getY()) / (end.getX() - start.getX() + 1e-9); //divide by zero case solved by 1e-9
    double b = start.getY() - (m * start.getX());

    return Line(m, b);
}

Line Line::buildByPointAndAngle(Point &start, double angle) {
    double m = tan(angle);
    double b = start.getY() - (m * start.getX());

    return Line(m, b);
}

double Line::getM() const {
    return m;
}


double Line::getB() const {
    return b;
}

LineSegment::LineSegment(Point start, Point end) : line(Line::buildByPoints(start, end)), start(new Point(start.getX(), start.getY())), end(new Point(end.getX(), end.getY())) {}

LineSegment::LineSegment(const Line &line, const Point &start, const Point &end) : line(line), start(new Point(start.getX(), start.getY())), end(new Point(end.getX(), end.getY())) {}

LineSegment::LineSegment(const LineSegment &copySegment): line(copySegment.line), start(new Point(copySegment.start->getX(), copySegment.start->getY())), end(new Point(copySegment.end->getX(), copySegment.end->getY())) {}

double LineSegment::length() {
    Point vector = (*end - *start);
    return vector.length();
}

Line LineSegment::getLine() {
    return line;
}

Point LineSegment::getStart() {
    return *start;
}

Point* LineSegment::getStartPtr() {
    return start;
}

Point LineSegment::getEnd() {
    return *end;
}

Point* LineSegment::getEndPtr() {
    return end;
}

// LineSegment::~LineSegment() {
//     delete start;
//     delete end;
// }
/*
bool Ellipse::inside(const Point &vector) {
    return pow(((vector.getX() - center.getX()) / radius_x), 2) +
           pow(((vector.getY() - center.getY()) / radius_y), 2) <= 1;
}

double Ellipse::size() {
    return _size;
}

LineSegment Ellipse::segmentIntersections(LineSegment &segment) {
    Line line = segment.getLine();
    LineSegment intersectionSegment = intersections(line);

    Point start = intersectionSegment.getStart();
    Point end = intersectionSegment.getEnd();

    if(start.getX() > end.getX()) {
        Point tmp = start;
        start = end;
        end = tmp;
    }

    Point segmentStart = segment.getStart();
    Point segmentEnd = segment.getEnd();

    if(segmentStart.getX() > segmentEnd.getX()) {
        Point tmp = segmentStart;
        segmentStart = segmentEnd;
        segmentEnd = tmp;
    }

    if(segmentStart.getX() > start.getX()) {
        start = segmentStart;
    }
    if(segmentEnd.getX() < end.getX()) {
        end = segmentEnd;
    }
    if(start.getX() > end.getX()) {
        start = end;
    }

    return LineSegment(segment.getLine(), start, end);
}


LineSegment Ellipse::intersections(Line &line) {
    double rx = radius_x * radius_x;
    double ry = radius_y * radius_y;

    double a = (1 / rx) + (line.getM() * line.getM() / ry);
    double b = (2 * line.getB() * line.getM() / ry) - (2 * center.getX() / rx) - (2 * center.getY() * line.getM() / ry);
    double c = (line.getB() * line.getB() / ry) - (2 * line.getB() * center.getY() / ry) + (center.getX() * center.getX() / rx) + (center.getY() * center.getY() / ry) - 1;

    // Solution using Quadratic equation -b +- sqrt(b^2 - 4ac)/2a
    // where ax^2 + bx + c = 0
    double discriminant = pow(b,2) - (4 * a * c);

    if (discriminant > 0){
        double x1 = ((-b) + sqrt(discriminant)) / (2 * a);
        double x2 = ((-b) - sqrt(discriminant)) / (2 * a);

        double y1 = (line.getM() * x1) + line.getB();
        double y2 = (line.getM() * x2) + line.getB();

        return LineSegment(line, Point(x1, y1), Point(x2, y2));
    } else {
        return LineSegment(line, Point(0, 0), Point(0, 0));
    }
}
*/

#endif

void Sync (Drone &A, Drone &B, double alpha, double dist, PLUME &plume)
{
    Point v;
    
    if (alpha > 0)
        v = PointUtil::vector (A.nabla + alpha - PI/2, DIST * epsilon);
    else
        v = PointUtil::vector (A.nabla + alpha + PI/2, DIST * epsilon);
    
    Point nextPosition = A.position + v;
    B.last = nextPosition;
    B.position = nextPosition;
    LineSegment dronemotion = LineSegment (B.last, B.position);
 //   if (abs(B.nabla - A.nabla) < 1e-9 &&  plume.getCross(dronemotion, B.nabla, alpha, dist, B.inside).second == 1)
   //     B.inside = B.inside ^ 1;
    B.nabla = A.nabla;
   // B.inside = A.inside;
    B.polytope.push_back (nextPosition);
    //ignoring angle turned during sync.
    
    return ;
}

// FIX ME: Verify this works for the critical path tracing
void Sync (Drone &A, Drone &B, double alpha, double dist, criticalPath &cp)
{
    Point v;
    
    if (alpha > 0)
        v = PointUtil::vector (A.nabla + alpha - PI/2, DIST * epsilon);
    else
        v = PointUtil::vector (A.nabla + alpha + PI/2, DIST * epsilon);

    
    Point nextPosition = A.position + v;
    B.last = nextPosition;
    B.position = nextPosition;


    Point BLast = B.last;
    Point BPosition = B.position;

    LineSegment dronemotion = LineSegment(BLast, BPosition);   

    B.nabla = A.nabla;
    B.polytope.push_back (nextPosition);

    //ignoring angle turned during sync.
    cout << "exiting sync" << endl;
    return ;
}

/*
string check (Point P){
    double A = majorAxis;
    double B = minorAxis;
    return (((P.x * P.x) / (A * A) + (P.y * P.y) / (B*B))  <= 1) ? "Inside " : "Outside ";
}
*/

void print_data(Drone A, Drone B)
{
    cout << "printing data..." << endl;
    int numPoints = A.polytope.size();
    
    cout << "printing data 1..." << endl;
    fprintf (out, "Pen b\n");
    
    for (int i = 0;i < numPoints; i ++)
        fprintf (out, "Line (%lf,%lf) (%lf,%lf)\n", A.polytope[i].x, A.polytope[i].y, A.polytope[(i+1)%numPoints].x, A.polytope[(i+1)%numPoints].y);
  
    numPoints = B.polytope.size();
  
    cout <<numPoints << endl;

//    fprintf (out, "Ellipse (%lf,%lf) %lf %lf \n", plume.ovals[0].center.x, plume.ovals[0].center.y, R, R/2);
 //   fprintf (out, "Ellipse (%lf,%lf) %lf %lf \n", plume.ovals[1].center.x, plume.ovals[1].center.y, R, R/2);

    cout << "printing data 2..." << endl;
    fprintf (out, "Pen r\n");
    
    for (int i = 0;i < numPoints; i ++)
        fprintf (out, "Line (%lf,%lf) (%lf,%lf)\n", B.polytope[i].x, B.polytope[i].y, B.polytope[(i+1)%numPoints].x, B.polytope[(i+1)%numPoints].y);
 
    cout << "done printing data..." << endl;
    return ;
}


bool CrossPlume (Drone &A, Drone &B, double alpha, PLUME &plume)
{
    Point start_pos = A.last;
   // if (abs(start_pos.x) < 2 && abs(start_pos.y) < 2)
     //   cout <<"start pos ... "<< start_pos.x << " " << start_pos.y <<" "<<check(A.position)<< endl;

    int crossing = A.inside;
    bool orient = true ;
    double alphainitial = alpha;
    bool endHere = false;
    
    int iterate = 1000;
    
    do{
        endHere = endHere || A.MoveDrone (alpha, epsilon * epsilon, plume, 0);
     //   B.MoveDrone (alpha, epsilon * epsilon, plume, 0);
        
        if (PointUtil::orientation(A.last, A.position, start_pos) == PointUtil::CLOCKWISE && alpha > 0)
            orient = false ;
        if (PointUtil::orientation(A.last, A.position, start_pos) == PointUtil::COUNTERCLOCKWISE && alpha < 0)
            orient = false ;
      //  if (crossing == A.inside)
        //    Sync (A,B,alpha, epsilon * epsilon, plume);
      //  else
        //    Sync (A,B,alphainitial, epsilon * epsilon, plume);
        
        if (alpha > 0)
            alpha += epsilon;
        else
            alpha -= epsilon;
        A.angleTurned += abs (alphainitial);
     //   cout << "testing cross plume ... "<< A.position.x << " " << A.position.y << endl;
     //   cout << "testing cross plume ... "<< B.position.x << " " << B.position.y << endl;
        iterate--;
        if (iterate < 0){
            print_data (A,B);
            exit (0);
        }
       // cout<<crossing<<" "<<A.inside<<" "<<A.droneIn<<endl;
    }while (crossing == A.inside && orient && !endHere);
        
    if (crossing == A.inside && !endHere){
        A.polytope.pop_back ();
        A.position = A.last;
        A.angleTurned -= abs (alphainitial);
        double dx = start_pos.getX () - A.position.getX();
        double dy = start_pos.getY () - A.position.getY();
        double gradient = atan2 (dy, dx);
       
    //    A.position = A.last;
     //   B.position = B.last;
        
        Point d1 = start_pos - A.position;
        Point motion;
        if (d1.length() > epsilon*epsilon)
            motion = PointUtil::vector (gradient, epsilon * epsilon);
        else
            motion = PointUtil::vector (gradient, d1.length());
        motion = A.position + motion;
        Point d2 = start_pos - motion;
     //   cout<<A.position.x<<" " <<A.position.y<<" "<<d1.length()<< " distances " << d2.length()<<endl;

        if (d1.length() < d2.length()){
            reverse (gradient);
        }
        A.angleTurned += changeGradient (A.nabla + alpha, gradient);
        A.nabla = gradient;
     //   B.nabla = gradient;
     //   cout << start_pos.x << " starting here " << start_pos.y << " "<<gradient<<" "<<check (start_pos) <<" "<<check(A.position)<<" "<<A.inside<< endl;
     //   cout << "testing cross plume2 ... "<< A.position.x << " " << A.position.y << endl;
     //   cout << "testing cross plume2 ... "<< A.last.x << " " << A.last.y << endl;
        int iter = 0;
        while (crossing == A.inside && !endHere){
            d1 = start_pos - A.position;
            if (d1.length() > epsilon * epsilon)
            {
                endHere = endHere || A.MoveDrone (0, epsilon*epsilon, plume, 0);
           //     Sync (A,B,alphainitial, epsilon * epsilon, plume);

            //    B.MoveDrone (0, epsilon * epsilon, plume, 0);
             //   B.MoveDrone (0, epsilon*epsilon, plume);
            }
            else{
                endHere = endHere || A.MoveDrone (0, d1.length(), plume, 0);
            //    Sync (A,B,alphainitial, d1.length(), plume);

             //   B.MoveDrone (0, d1.length(), plume, 0);

            //    B.MoveDrone (0, d1.length()/2, plume);
            }
            
         //   if (abs(A.position.x) > 2 || abs(A.position.y) > 2)
           //     break ;
            ++iter;
            if (iter > 10000)
            {
                cout<<"Iterations exceeding ..."<<endl;
                print_data(A,B);
                exit (0);
            }
        //    Sync2 (A,B, alpha);
            //cout << "testing cross plume2 ... "<< A.position.x << " " << A.position.y << endl;
      //      cout << "testing cross plume2 ... "<< B.position.x << " " << B.position.y << endl;
        }
      //  B.nabla = A.nabla;
     //   Sync (A,B,alphainitial, epsilon * epsilon, plume);
    }
    
    return endHere;
}


// FIX ME: Make this work for the critical path tracing
bool CrossCriticalPath(Drone &A, Drone &B, double alpha, criticalPath &cp)
{
    Point start_pos = A.last; 

    int crossing = A.side; 
    bool orient = true ;
    double alphainitial = alpha;
    bool endHere = false;
    
    int iterate = 1000;
    

    do{
        A.MoveDrone (alpha, epsilon, cp.diffepsilon, 0);
        CrossData crossData = cp.getCross(A, B, alpha, epsilon);
        if(crossData.second){
            vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
            A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
            B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);
        }
        cout << "drone side? " << A.side << endl;
        endHere = endHere || cp.foundSource(A, B);
        
        // FIX ME: check if this is determining the correct orientation?
        Point last = A.last;
        Point position = A.position;
        if (PointUtil::orientation(last,position, start_pos) == PointUtil::CLOCKWISE && alpha > 0)
            orient = false ;
        if (PointUtil::orientation(last, position, start_pos) == PointUtil::COUNTERCLOCKWISE && alpha < 0)
            orient = false ;

        if (alpha > 0)
            alpha += epsilon;
        else
            alpha -= epsilon;

        A.angleTurned += abs (alphainitial);

        iterate--;
        if (iterate < 0){
            print_data (A,B);
            exit (0);
        }
    }while (crossing == A.side && orient && !endHere);
        
    if (crossing == A.side && !endHere){
        A.polytope.pop_back (); 
        A.polytope.pop_back();
        A.position = A.last;
        A.angleTurned -= abs (alphainitial);
        double dx = start_pos.getX () - A.position.getX();
        double dy = start_pos.getY () - A.position.getY();
        double gradient = atan2 (dy, dx);
        
        Point pos = A.position;
        Point d1 = start_pos - pos;
        Point motion;
        if (d1.length() > epsilon*epsilon)
            motion = PointUtil::vector (gradient, epsilon * epsilon);
        else
            motion = PointUtil::vector (gradient, d1.length());
        motion = pos + motion;
        Point d2 = start_pos - motion;

        if (d1.length() < d2.length()){
            reverse (gradient);
        }

        A.angleTurned += changeGradient (A.nabla + alpha, gradient);
        A.nabla = gradient;

        int iter = 0;
        while (crossing == A.side && !endHere){
            Point new_pos = A.position; 
            d1 = start_pos - new_pos; 
            if (d1.length() > epsilon * epsilon)
            {
                A.MoveDrone(0, epsilon*epsilon, cp.diffepsilon, 0);
                CrossData crossData = cp.getCross(A, B, alpha, epsilon);
                if(crossData.second){
                    vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
                    A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
                    B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);
                }
                cout << "Drone side now " << A.side << endl;
                endHere = endHere || cp.foundSource(A, B); 
            }
            else{
                A.MoveDrone(0, d1.length(), cp.diffepsilon, 0);
                CrossData crossData = cp.getCross(A, B, alpha, epsilon);
                if(crossData.second){
                    vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
                    A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
                    B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);
                }
                cout << "Drone side now " << A.side << endl;
                endHere = endHere || cp.foundSource(A, B);
            }
            
            ++iter;
            if (iter > 10000)
            {
                cout<<"Iterations exceeding ..."<<endl;
                print_data(A,B);
                exit (0);
            }
        }
    }
    
    return endHere;
}

/*
double estimateArea (vector<Point> polygon)
{
    double area = 0 ;
    int n = polygon.size ();
    
    for (int i = 0;i < n; i ++)
        area += polygon[i].x * polygon [(i+1)%n].y - polygon[i].y* polygon[(i+1)%n].x;
    
    area /=2 ;
    
    return area ;
}
*/

void sketch_algorithm (double alpha)
{
    cout << "Running Sketch Algorithm for eapsilon = " << epsilon << endl;
    
    
    Drone A (drone_start_A, drone_start_A, 1, 0, true);
    Drone B (drone_start_B, drone_start_B, 2, 0, false);

    int TOKEN = 2;
    
    bool loopEnd = false ;

    Point startPoint = {(drone_start_A.x + drone_start_B.x) / 2, (drone_start_A.y + drone_start_B.y) / 2};
    double res = epsilon*epsilon; // how much resolution should we calculate contourlines at?
    criticalPath cp(startPoint, epsilon, res);
    
    
    do{
        int iter = 0;


        while ((A.numCross + B.numCross == 0 || 3 == A.side + B.side) && !loopEnd)
        {
            cout << "inside loop " << iter << endl;
            ++iter;
            if (iter > 10000)
            {
                cout<<"Iterations exceeding ..."<<endl;
                print_data(A,B); 
                exit (0);
            }
            
            A.MoveDrone(alpha, epsilon, cp.diffepsilon, 1);
            B.MoveDrone(alpha, epsilon, cp.diffepsilon, 1);
            loopEnd = loopEnd || cp.foundSource(A, B);
            
            CrossData crossData = cp.getCross(A, B, alpha, epsilon);
            if(crossData.second){
                // cout << "numcross A" << A.numCross << endl;
                // cout << "numcross B" << B.numCross << endl;
                // cout << "A side" << A.side << endl;
                // cout << "B side" << B.side << endl;

                // cout << "crossed; learning gradient..." << endl;
                vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
                A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
                B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);

            }

            
        }


        if (A.side + B.side != 3)
        {

            // If A crosses
            if (A.side == 2 && B.side == 2)
            {
                alpha = -epsilon;
            //    B.nabla = A.nabla;

                loopEnd = loopEnd || CrossCriticalPath (B,A, alpha, cp);
                cout << "about to sync" << endl;
                Sync (B,A,alpha, epsilon, cp); // FIX ME
                A.nabla = B.nabla;
              
 //               cout << "testing Sync A... "<< A.position.x << " "<<A.position.y <<" "<<alpha<<" "<<A.nabla<< endl;
 //               cout << "testing Sync B... "<< B.position.x << " "<<B.position.y <<" "<<alpha<<" "<<B.nabla<< endl;
            }
            else // B crosses
            {
              alpha = epsilon;
              loopEnd = loopEnd || CrossCriticalPath (A,B, alpha, cp);
              cout << "about to sync" << endl;
              Sync (A,B,alpha, epsilon, cp);
              B.nabla = A.nabla;
                
   //             cout << "testing Sync B... "<< B.position.x << " "<<B.position.y <<" "<<alpha<<" "<<B.nabla<< endl;
   //             cout << "testing Sync A... "<< A.position.x << " "<<A.position.y <<" "<<alpha<<" "<<A.nabla<< endl;
            
            }
        }
    }while (!loopEnd);
    

    if (!A.polytope.empty()) {
        cout <<"Initial crossing with A is " << A.polytope[0].getX() << " " << A.polytope[0].getY() << endl;
    } else {
        cout << "Initial crossing with A is not available as polytope is empty." << endl;
    }
    cout << "angle turned by A is " << " " << A.angleTurned << endl;
    cout << "distance traversed by A is "<< " " << A.distTraversed << endl;
    if (!A.polytope.empty()) {
        cout << "area estimated by A is " << " " << estimateArea (A.polytope) << endl;
        areas.push_back (estimateArea (A.polytope));
    } else {
        cout << "area estimated by A is not available as polytope is empty." << endl;

    lengths.push_back (A.distTraversed);
    angles.push_back (A.angleTurned);
    cout << "actual area is  " << PI * majorAxis * minorAxis << endl;
    }

    print_data (A,B);
    
    return ;
}   

void test_infrastructure()
{
    gaussianCenter.clear();
    gaussianVar.clear();
    double alpha = 1.47; // slightly off pi/2 to try to cause early crossing
    num = 4;
    CROSSBOUND = 100;
    majorAxis = 0.25;
    minorAxis = 0.25;
    DIST = sqrt (49);
    THRESHOLD = exp (-majorAxis*majorAxis);
   
    drone_start_B = Point (1 + DIST*epsilon*0.5,-2.38);
    drone_start_A = Point (1 - DIST*epsilon*0.5,-2.38);

    drone_start_BB = Point (1 + DIST*epsilon*0.5,-2.38);
    drone_start_AB = Point (1 - DIST*epsilon*0.5,-2.38);
    
    // data/gaussian_contours_data_7.txt  
    gaussianCenter.push_back(Point(1,1));
    gaussianCenter.push_back(Point(-2,0));
    gaussianCenter.push_back(Point(1,0));
    gaussianCenter.push_back(Point(4,0));
    
    gaussianVar.push_back(Point(0.6,0.6));
    gaussianVar.push_back(Point(0.8,1.0));
    gaussianVar.push_back(Point(0.9,1.8));
    gaussianVar.push_back(Point(0.5,0.5));

    cout<<"Initializing test... "<<endl;
    cout<<"Epsilon "<<epsilon<<endl;
    cout<<"Initial direction "<<alpha<<endl;
    cout<<"Number of Gaussians "<<num<<endl;
    cout<<"Least difference between starting and end point "<<INF<<endl;
    cout<<"Minimum crossings before checking termination "<<CROSSBOUND<<endl;
    cout<<"Minimum distance factor between drones and minimum distance "<<DIST<<" " <<DIST * epsilon<<endl;
    cout<<"Concentration THRESHOLD "<<THRESHOLD<<endl;
    cout<<"Drone A starting point "<<drone_start_A.x<<" "<<drone_start_B.y<<endl;
    cout<<"Drone B starting point "<<drone_start_B.x<<" "<<drone_start_A.y<<endl;
    
    cout<<"Centers of gaussians "<<endl;
    
    for (int i =0 ; i < num; i ++)
        cout<<gaussianCenter[i].x<<" "<<gaussianCenter[i].y<<endl;
        
    sketch_algorithm(alpha);
}

