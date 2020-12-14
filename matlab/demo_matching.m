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

clear all; close all; dbstop error;

% read images from file
I1p = int16(imread('../img/I1p.png'));
I2p = int16(imread('../img/I2p.png'));
I1c = int16(imread('../img/I1c.png'));
I2c = int16(imread('../img/I2c.png'));

% init matcher
matcherMex('init');

% push back images
matcherMex('push',I1p',I2p'); 
matcherMex('push',I1c',I2c'); 

% match images
p_matched = matcherMex('match',int32(5000),5000,5000);

% close matcher
matcherMex('close');

% show matching results
figure,plotMatch(I1c,p_matched);
