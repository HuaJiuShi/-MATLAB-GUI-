function y = isbw(x)
%ISBW Return true for binary image.
%   FLAG = ISBW(A) returns 1 if A is a binary image and 0
%   otherwise.
%
%   ISBW uses these criteria to decide if A is a binary image:
%
%   - if A is of class double, all values must be either 0 or 1,
%     and the number of dimensions of A must be 2.
%
%   - If A is of class uint8, its logical flag must be on, and
%     the number of dimensions of A must be 2.
%
%   Note that a four-dimensional array that contains multiple
%   binary images returns 0, not 1.
%
%   Class Support
%   -------------
%   A can be of class uint8 or double.
%
%   See also ISIND, ISGRAY, ISRGB.

%   Clay M. Thompson 2-25-93
%   Copyright 1993-1998 The MathWorks, Inc.  All Rights Reserved.
%   $Revision: 5.8 $  $Date: 1997/11/24 15:35:45 $

y = ndims(x)==2;
if isa(x, 'double') & y
   if islogical(x)
      % At first just test a small chunk to get a possible quick negative  
      [m,n] = size(x);
      chunk = x(1:min(m,10),1:min(n,10));         
      y = ~any(chunk(:)~=0 & chunk(:)~=1);
      % If the chunk is a binary image, test the whole image
      if y
         y = ~any(x(:)~=0 & x(:)~=1);
      end
   else
      y = 0;
   end
elseif isa(x, 'uint8') & y
   y = islogical(x);
end

y = logical(double(y));




