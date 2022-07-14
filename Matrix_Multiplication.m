# Matrix multiplication program
# It takes 2 Euler Representation cartesiantorx [x y z rx ry rz] and returns also Euler representation of A*B
# This is meant to simulate the vision offset
# Created by:     Mohanad Sawas
# Created on:     July-2022
# Resource:       Is based on the following document (c:\Users\90500477\Documents\iRVision\Technologies\Mathematics\Euler_Representation.pdf)



clc
format short g # to show the result without e format
function MatForm= cartesian_to_matrix (CartesianForm)
  rx= rotx(CartesianForm(1,4));
  ry= roty(CartesianForm(1,5));
  rz= rotz(CartesianForm(1,6));
  r=rz*ry*rx;
  trans=[CartesianForm(1,1); CartesianForm(1,2);CartesianForm(1,3)];
  MatForm=[r, trans];
  last_row=[0 0 0 1];
  MatForm=[MatForm; last_row];
endfunction


function MatRes= cartesian_multiplication (Cartesian1, Cartesian2)
  Mat1= cartesian_to_matrix(Cartesian1);
  Mat2= cartesian_to_matrix(Cartesian2);
  MatRes=Mat1*Mat2;
endfunction

function [CartesianForm_1,CartesianForm_2]= matrix_to_cartesiantor (MatForm)
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
  CartesianForm_1(1,1)=CartesianForm_2(1,1)=round(100*x)/100;
  CartesianForm_1(1,2)=CartesianForm_2(1,2)=round(100*y)/100;
  CartesianForm_1(1,3)=CartesianForm_2(1,3)=round(100*z)/100;
  CartesianForm_1(1,4)=round(100*rx1)/100;
  CartesianForm_1(1,5)=round(100*ry1)/100;
  CartesianForm_1(1,6)=round(100*rz1)/100;
  CartesianForm_2(1,4)=round(100*rx2)/100;
  CartesianForm_2(1,5)=round(100*ry2)/100;
  CartesianForm_2(1,6)=round(100*rz2)/100; # round(100 * value) / 100
  endfunction




# multiply 2 cartesiantors
function ResCartesiant= multiply_two_cartesiantors(Cartesian1, Cartesian2)
  MatForm= cartesian_multiplication (Cartesian1, Cartesian2);
  ResCartesiant=matrix_to_cartesiantor (MatForm);
    endfunction

# function to calculate Voffset out of Reference position and Found Position
function VOffset= calc_VOffset(RF,FP)
  #Voffset= FP:inv(RF)
  RF_mat=cartesian_to_matrix(RF);
  InvRF_mat=inv(RF_mat);
  InvRF=matrix_to_cartesiantor(InvRF_mat);
  VOffset_mat=cartesian_multiplication(FP,InvRF);
  VOffset=matrix_to_cartesiantor(VOffset_mat);
  endfunction


function InvCartesiant=inverse_cartesiantor(Cartesiantor)
  MatForm= cartesian_to_matrix (Cartesiantor);
  InvMatForm=inv(MatForm);
  InvCartesiant=matrix_to_cartesiantor (InvMatForm);
  endfunction

function PointOffset= calc_point_offset(RF,FP)
  #Point offset=inv(RF):FP
  RF_mat=cartesian_to_matrix(RF);
  InvRF_mat=inv(RF_mat);
  InvRF=matrix_to_cartesiantor(InvRF_mat);
  PointOffset_mat=cartesian_multiplication(InvRF,FP);
  PointOffset=matrix_to_cartesiantor(PointOffset_mat);
  endfunction















  # Hi user, Enter A and B
RF=[0.855	-0.622	-0.056	0.946	-0.011	-0.007];
FP=[11.2	9.5	-0.9	0.9	5.2	0];
disp('Calculated Voffset is: ')
Voffset=calc_VOffset(RF,FP)

disp('Calculated Point Offset is: ')
PointOffset=calc_point_offset(RF,FP)


P2=[-1.318	-0.425	1.774	-179.998	-5.151	-180];
P2_Shifted=[9.204	9.651	1.115	-179.98	-10.327	179.999];
InvP2=inverse_cartesiantor(P2);
offsetPickPose=multiply_two_cartesiantors(InvP2,P2_Shifted)
