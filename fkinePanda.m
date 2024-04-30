function pose = fkinePanda (kinematicModel,q, frame)
    M = kinematicModel.M
    S=kinematicModel.S
    pose = fkine(S,M,q, frame);

end 
