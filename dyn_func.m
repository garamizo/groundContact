function out1 = dyn_func(in1,in2,in3,in4)
%DYN_FUNC
%    OUT1 = DYN_FUNC(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    05-Dec-2016 10:35:58

f1 = in3(1,:);
f2 = in3(2,:);
fa1 = in4(1,:);
fa2 = in4(2,:);
q1 = in1(1,:);
q2 = in1(2,:);
q1d = in2(1,:);
q2d = in2(2,:);
t2 = sin(q2);
t3 = cos(q2);
t4 = sin(q1);
t5 = q1+q2;
t6 = cos(t5);
t7 = q1d.^2;
t8 = t2.^2;
t9 = t8.*2.7e5;
t10 = t9-4.84003e5;
t11 = 1.0./t10;
t12 = q1d.*3.224e5;
t13 = q2d.^2;
t14 = t3.*t13.*1.209e5;
t15 = sin(t5);
t16 = cos(q1);
t17 = fa1.*t16.*4.836e4;
t18 = fa2.*t4.*4.836e4;
t19 = q1d.*q2d.*t3.*2.418e5;
t20 = fa2.*t2.*t6.*7.2e4;
out1 = [t11.*(f1.*-3.224e4+f2.*3.224e4-q2d.*3.224e5-t4.*3.558087e6+t12+t14+t17+t18+t19+t20+f2.*t2.*7.2e4-q2d.*t2.*7.2e5-t2.*t6.*1.7658e6+t3.*t7.*1.209e5-fa1.*t2.*t15.*7.2e4+t2.*t3.*t7.*2.7e5);-t11.*(f1.*-3.224e4+f2.*3.2048e5-q2d.*3.2048e6-t4.*3.558087e6-t6.*7.069086e6+t12+t14+t17+t18+t19+t20-f1.*t2.*7.2e4+f2.*t2.*1.44e5+fa2.*t6.*2.8824e5-fa1.*t15.*2.8824e5+q1d.*t2.*7.2e5-q2d.*t2.*1.44e6-t2.*t4.*7.9461e6-t2.*t6.*1.7658e6+t3.*t7.*1.2018e6+fa2.*t2.*t4.*1.08e5-fa1.*t2.*t15.*7.2e4+fa1.*t2.*t16.*1.08e5+t2.*t3.*t7.*5.4e5+t2.*t3.*t13.*2.7e5+q1d.*q2d.*t2.*t3.*5.4e5)];
