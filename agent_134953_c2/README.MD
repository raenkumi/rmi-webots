# C2 AGENT - 134953

## To the Professors

I wasn't able to finish this completely for the deadline, because I've parted with my colleague and decided to start the project from scratch because his approach was confusing. I guess with 3 more working days over this project I could have met the deliverables of C2 and C3, but I've been overworked by my employers recently.

Nevertheless, instead of cheating with AI I opted to deliver what I was able to do alone. 

This zip includes the robot controller model, a mapping model, a PD controller for the bot and basic navigation/pathfinding. This agent performs poorly and is missing the map outputs and the entire C3 implementation and output.

I'll do my presentation anyways, since I've still made my research to try building this, but just wasn't able to finish in time.

## How to run

Copy all contents from this .zip to the webots/controller/C2_cClient folder, then build over webots with the gear button. This follows the same C1_cClient config, without any special libs or linkers.

You might need to point the C2_Supervisor to the C2_cClient aswell, since only the C2_pClient exists.