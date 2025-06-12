
#ifndef LEGACY
double changeGradient(double angleA, double angleB){
  assert( abs(angleA) < (2*PI) );
  assert( abs(angleB) < (2*PI) );
  
  double dangle = angleB-angleA;
  int magnitude = abs(dangle) > (2*PI);
  int sign = ((dangle < 0) << 1) -1;//plus or minus 1
  dangle += (sign * magnitude * 2) * PI;
  return dangle;
}
#endif //LEGACY
//the absolute value is wrong if we really are dealing with polar coordinates
//the approximation of pi we are using could cause us to accumulate a lot of error
//when computing successive rotations of the gradient

#ifdef LEGACY
double changeGradient(double angleA, double angleB)
{
  //-theta = 2pi - theta
    return abs(angleA - angleB); //check this again for bug.
}
#endif //LEGACY

void reverse(double &gradient)
{
    if (gradient < 0)
        gradient = gradient + PI;
    else if (gradient > 0)
        gradient = gradient - PI;
    else
        gradient = 0 ;
    return ;
}


int checkin(Param* p, Point P){
    double A = p->majorAxis;
    double B = p->minorAxis;
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

vector<double> get_gradient(vector<vector<double> > A, vector<double> x, vector<double> b)
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
  //  x.push_back (-2*(points[1].x-p->gaussianCenter[id].x) * fval[1]);
  //  x.push_back (-2*(points[1].y-p->gaussianCenter[id].y) * fval[1]);
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

double gaussian(Param* p, Point input, int gaussianId)
{
    double X = input.x - p->gaussianCenter[gaussianId].x;
    double Y = input.y - p->gaussianCenter[gaussianId].y;
    double varX = p->gaussianVar[gaussianId].x;
    double varY = p->gaussianVar[gaussianId].y;
    return exp( - (X * X / (2*varX)) - (Y * Y/(2*varY)));
}

vector<double> getGaussian(Param* p, vector<Point> points)
{
    vector<double> fval;
    
    for (int i = 0; i < points.size(); i ++)
    {
        double sum = 0;
        for (int j = 0; j < p->gaussianCenter.size(); j++)
            sum = sum + gaussian(p, points[i], j);
        fval.push_back (sum);
    }
    
    return fval;
}

double getGaussian(Param* p, Point point)
{
    double fval;
    double sum = 0;
    for (int j = 0; j < p->gaussianCenter.size(); j++){
        sum = sum + gaussian(p, point, j);
    } 
    fval = sum;
    return fval;
}

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
vector<Point> getGaussianContours(Param* p,
                                  double level,
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

            double f1 = getGaussian(p, p1);
            double f2 = getGaussian(p, p2);
            double f3 = getGaussian(p, p3);
            double f4 = getGaussian(p, p4);

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

//what's this supposed to do?
vector<double> get_Gaussian_vector(Param* p, vector<Point> points, int id)
{
    vector<double> x;
    vector<double> fval = getGaussian(p, points);
    //below is buggy, does not incorporate variance
    x.push_back (-2*(points[1].x-p->gaussianCenter[id].x) * fval[1]);
    x.push_back (-2*(points[1].y-p->gaussianCenter[id].y) * fval[1]);
    return x;
}

//this looks wrong...
double gradient_modulo(double gradient)
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

vector<double> gradient_LSQ(Param* p, vector<Point> points)
{
    vector<double> fval = getGaussian(p, points);
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
    Param* p,
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
    double fb = getGaussian(p, pointToSolve);
    // cout << "concentration at pts: " << fb << endl;

    std::vector<std::vector<double>> A;
    std::vector<double> b;

    for (auto& point : surroundingPoints) {
        double x = point.x;
        double y = point.y;
        double f = getGaussian(p, point);
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
    Param* p,
    std::vector<Point> surroundingPoints,
    Point& pointToSolve,
    std::vector<double>& gradientAtPoint) {

    double xE = pointToSolve.x;
    double yE = pointToSolve.y;
    double fxe = gradientAtPoint[0];
    double fye = gradientAtPoint[1];
    double fe = getGaussian(p, pointToSolve);

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
        double f = getGaussian(p, surroundingPoints[i]); 

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
    Param* p,
    std::vector<Point> surroundingPoints,
    Point& pointToSolve)
{
    // Calculate the gradient at pointB using three points.
    std::vector<double> gradB = concentration_gradient_LSQ(p, surroundingPoints, pointToSolve); 

 
    // Calculate the Hessian using nine points, the computed gradient, and the concentration function.
    std::vector<std::vector<double>> H = hessian_LSQ(p, surroundingPoints, 
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


#ifndef LEGACY
double getAngle(vector<double> A)
{
    
    int magnitude = (A[0] != 0) + (A[1] != 0);
    assert(magnitude>0);

    double normA = sqrt ( (A[0] * A[0]) + (A[1] * A[1]) );
    int max = (A[1]*A[1]) > (A[0]*A[0]);
    
    switch(magnitude){

      case 1://vector has one nonzero component
      case 2://vector has one nonzero component
        switch(max){
          case 0://A[1] is 0
            return acos(A[0]/normA); //
          case 1://A[0] is 0
            return asin(A[1]/normA); //
        }
    }
    assert(0);//this should never be reached
    return 0.0; //can't measure angle of zero vector
}
#endif //LEGACY

#ifdef LEGACY
//previous implementation
double getAngle(vector<double> A)
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
#endif //LEGACY

/*
 * vectors a and b are orthogonal if a.b=0
 * we have 2 degrees of freedom so we choose a
 * new coordinate st magnitude of the denominator
 * in the problem is maximized which minimizes error propagation
 * */
Point get_orthogonal_vector(double x, double y){
  int magnitude = (x != 0) + (y != 0);
  assert(magnitude > 0);
  int max = abs(y) > abs(x);
  double nx, ny;
  if(max){
    nx = x;
    ny = (-1 * x * nx) / y;
  }
  else{
    ny = y;
    nx = (-1 * y * ny) / x;
  }
  return Point(nx,ny);
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
/*************************************************************/

string check(Param* p, Point P){
    double A = p->majorAxis;
    double B = p->minorAxis;
    return (((P.x * P.x) / (A * A) + (P.y * P.y) / (B*B))  <= 1) ? "Inside " : "Outside ";
}

/***************************************************/

double estimateArea(vector<Point> polygon)
{
    double area = 0 ;
    int n = polygon.size();
    
    for (int i = 0;i < n; i ++)
        area += polygon[i].x * polygon [(i+1)%n].y - polygon[i].y* polygon[(i+1)%n].x;
    
    area /=2 ;
    
    return area ;
}

