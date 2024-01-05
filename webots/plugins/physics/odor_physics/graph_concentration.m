% simulation->mOdorModel->WriteConcentration(file, 0, Point3(0, 0, -2), Point3(16, 1, 2), Point3(0.01, 2, 0.01) -> a is 400x1601
function concentrations=graph_concentration(file)

% One line per z, looped over y (innermost is x)

% This file contains a 400x1601 wide map of the ground level concentrations
% after 20 timesteps.
concentrations=load(file);
set(figure, 'name', 'odor profile');
set(surf(concentrations), 'edgecolor','none');

title 'Odor profile';
zlabel('Odor concentration');
xlabel('x');
ylabel('z');
