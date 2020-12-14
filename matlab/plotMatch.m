% Copyright 2010. All rights reserved.
% Institute of Measurement and Control Systems
% Karlsruhe Institute of Technology, Germany

% This file is part of libviso.
% Authors: Bernd Kitt, Andreas Geiger

% libviso is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 2 of the License, or any later version.

% libviso is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% libviso; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA 


function plotMatch(I,p_matched,inliers)

if nargin<3
  inliers = 1:size(p_matched,2);
end

p_matched = double(p_matched)';
disp = p_matched(:,1)-p_matched(:,3);
max_disp = max(disp(inliers));

% show image and plot line
cla,imshow(uint8(I)),hold on;
for i=1:size(p_matched,1)
  c = abs(disp(i)/max_disp);
  col = [c 1-c 0];
  if ~any(inliers==i)
    col = [0 0 1];
  end
  line([p_matched(i,1) p_matched(i,5)], ...
       [p_matched(i,2) p_matched(i,6)], 'Color', col,'LineWidth',1);
  plot(p_matched(i,5),p_matched(i,6),'s', 'Color', col,'LineWidth',2,'MarkerSize',2);
end

