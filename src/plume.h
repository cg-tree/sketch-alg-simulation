

struct PLUME{
    vector<Ellipse> ovals;
    
    PLUME ()
    {
    }
    
    int pointinEllipse(Param* p, Point P, Ellipse oval)
    {
        double A = p->majorAxis;
        double B = p->minorAxis;
        P.x -= oval.center.x;
        P.y -= oval.center.y;
        double leftHandside = (P.x * P.x) / (A * A) + (P.y * P.y) / (B*B);
        if (leftHandside <= 1)
            return 1;
        else
            return 0;
    }
    
    double concentration(Param* p, Point P)
    {
        double sum = 0;
        
        for (int i = 0;i < p->num; i++){
            sum = sum + gaussian(p, P, i);
        }
        
        return sum ;
    }
    
  
    CrossData getCross(Param* p, LineSegment segment, double nabla, double alpha, double dist, int inside)
    {
        Point init = *segment.start;
        Point fini = *segment.end;
        Point curr = init;
        
        Point motion = PointUtil::vector(nabla + p->alpha, dist/100);
        
        for (int i = 0;i < 100; i ++){
            curr = curr + motion;
            if (inside && concentration(p, curr) < p->THRESHOLD){
                return CrossData(curr, 1);
            }
            else if (!inside && concentration(p, curr) > p->THRESHOLD)
                return CrossData(curr, 1);
        }
        
        
       // cout <<"Exception did not cross!"<<endl;
       // exit (0);
        
        return CrossData( Point (0,0), 0 );
    }
  
};

