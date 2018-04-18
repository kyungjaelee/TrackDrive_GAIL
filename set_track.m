function track = set_track(track, tracktype)

switch tracktype
     case 'simple'
          track  = add_segment(track, 'straight', track.width, 40000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
          track  = add_segment(track, 'left_turn', track.width, 20000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
          track  = add_segment(track, 'straight', track.width, 40000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
     case 'complex'
          track  = add_segment(track, 'straight', track.width, 40000);
          track  = add_segment(track, 'left_turn', track.width, 20000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
          track  = add_segment(track, 'right_turn', track.width, 30000);
          track  = add_segment(track, 'right_turn', track.width, 30000);
          track  = add_segment(track, 'left_turn', track.width, 20000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
          track  = add_segment(track, 'right_turn', track.width, 40000);
          track  = add_segment(track, 'right_turn', track.width, 20000);
     case 'demonstration'
          track  = add_segment(track, 'straight', track.width, 40000);
          track  = add_segment(track, 'straight', track.width, 20000);
          track  = add_segment(track, 'straight', track.width, 20000);
          track  = add_segment(track, 'straight', track.width, 20000);
     case 'auto_collect'
          track  = add_segment(track, 'straight', track.width, 40000);
          track  = add_segment(track, 'straight', track.width, 20000);
          track  = add_segment(track, 'straight', track.width, 20000);
          track  = add_segment(track, 'straight', track.width, 20000);
          track  = add_segment(track, 'straight', track.width, 20000);
          track  = add_segment(track, 'straight', track.width, 20000);
     otherwise
          fprintf('???\n');
end
