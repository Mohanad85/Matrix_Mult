# Matrix multiplication program
# It takes 2 Euler Representation vectorx [x y z rx ry rz] and returns also Euler representation of A*B
# This is meant to simulate the vision offset
# Created by:     Mohanad Sawas
# Created on:     July-2022
# Resource:       Is based on the following document (c:\Users\90500477\Documents\iRVision\Technologies\Mathematics\Euler_Representation.pdf)



clc
format short g # to show the result without e format
function MatForm= vec_to_matrix (VecForm)
  rx= rotx(VecForm(1,4));
  ry= roty(VecForm(1,5));
  rz= rotz(VecForm(1,6));
  r=rz*ry*rx;
  trans=[VecForm(1,1); VecForm(1,2);VecForm(1,3)];
  MatForm=[r, trans];
  last_row=[0 0 0 1];
  MatForm=[MatForm; last_row];
endfunction


function MatRes= vec_multiplication (Vec1, Vec2)
  Mat1= vec_to_matrix(Vec1);
  Mat2= vec_to_matrix(Vec2);
  MatRes=Mat1*Mat2;
endfunction

function [VecForm_1,VecForm_2]= matrix_to_vector (MatForm)
  x=MatForm(1,4);
  y=MatForm(2,4);
  z=MatForm(3,4);
  r11= MatForm(1,1); r12= MatForm(1,2); r13= MatForm(1,3);
  r21= MatForm(2,1); r22= MatForm(2,2); r23= MatForm(2,3);
  r31= MatForm(3,1); r32= MatForm(3,2); r33= MatForm(3,3);

  if ((r31!=1) & (r31!=-1))
    ry1=(-asin(r31))*180/pi;
    ry2=(pi-ry1)*180/pi;
    rx1=(atan2((r32/cos(ry1)),(r33/cos(ry1))))*180/pi;
    rx2=(atan2((r32/cos(ry2)),(r33/cos(ry2))))*180/pi;
    rz1=(atan2((r21/cos(ry1)),(r11/cos(ry1))))*180/pi;
    rz2=(atan2((r21/cos(ry2)),(r11/cos(ry2))))*180/pi;
  else
    rz1=rz2=0
    if (r31==-1)
      ry1=ry2=(pi/2)*180/pi
      rx1=rx2=(rz1+atan2(r12,r13))*180/pi
    else
      ry1=ry2=(-pi/2)*180/pi
      rx1=rx2=(-rz1+atan2(-r12,-r13))*180/pi
    endif

  endif
  VecForm_1(1,1)=VecForm_2(1,1)=round(100*x)/100;
  VecForm_1(1,2)=VecForm_2(1,2)=round(100*y)/100;
  VecForm_1(1,3)=VecForm_2(1,3)=round(100*z)/100;
  VecForm_1(1,4)=round(100*rx1)/100;
  VecForm_1(1,5)=round(100*ry1)/100;
  VecForm_1(1,6)=round(100*rz1)/100;
   VecForm_2(1,4)=round(100*rx2)/100;
  VecForm_2(1,5)=round(100*ry2)/100;
  VecForm_2(1,6)=round(100*rz2)/100; # round(100 * value) / 100
  endfunction



# Hi user, Enter A and B

A=[1 2 3 90 180 30];
B=[16 3 4 -45 90 45];
C=vec_multiplication(A,B);
[D1,D2]=matrix_to_vector(C)
