function pid_derivate_test()
%PID_DERIVATE_TEST system to test that the derivative term works well

dt_ = 0.1;
dt_all = dt_*[1:1000]
x = sin(dt_all);
x_n = x + (rand(size(x)) - 0.5)

close all
for i=20:1:1000
    dt = dt_all(i-19:i);
    dx = x(i-19:i);
    dx_n = x_n(i-19:i);
    
    subplot(2,1,1)
    plot(dt,dx,dt,dx_n)
    
    
    m = (dx(end) - dx(end-1))/dt_
    m_n = (dx_n(end) - dx_n(end-1))/dt_
    
    vector_dt = dt - dt(1);
    
    m_n = (sum(vector_dt.*dx) - sum(vector_dt)*sum(dx)/length(dx)) / ( sum(dx.^2) - sum(dx)^2/length(dx))
    
    subplot(2,1,2)
    plot(dt, m*(dt -dt(1)), dt, m_n*(dt -dt(1)))
    ylim([-2 2])
    
    pause(dt_)
end
    
end

