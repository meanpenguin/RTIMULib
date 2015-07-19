#ifndef RunningAverage_h

#define RunningAverage_h

#define _stdev(cnt, sum, ssq) sqrt((((double)(cnt))*ssq-pow((double)(sum),2)) / ((double)(cnt)*((double)(cnt)-1)))

class moving_average {
private:
    boost::circular_buffer<int> *q;
    float sum;
    float ssq;
public:
    moving_average(int n)  {
        sum=0;
        ssq=0;
        q = new boost::circular_buffer<int>(n);
    }
    ~moving_average() {
        delete q;
    }
    void push(double v) {
        if (q->size() == q->capacity()) {
            float t=q->front();
            sum-=t;
            ssq-=t*t;
            q->pop_front();
        }
        q->push_back(v);
        sum+=v;
        ssq+=v*v;
    }
    float size() {
        return q->size();
    }
    float mean() {
        return sum/size();
    }
    float stdev() {
        return _stdev(size(), sum, ssq);
    }

};

#endif