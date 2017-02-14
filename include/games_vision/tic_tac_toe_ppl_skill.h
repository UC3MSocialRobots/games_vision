/*!
  \file        tic_tac_toe_ppl_skill.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/21

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.

\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription

\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication

 */

#ifndef TIC_TAC_TOE_PPL_SKILL_H
#define TIC_TAC_TOE_PPL_SKILL_H

#include "games_vision/tic_tac_toe_skill.h"
// people_msgs
#include "vision_utils/ppl_user_tracker.h"

class TicTacToePPLSkill : public TicTacToeSkill {
public:
  typedef people_msgs::People PPL;
  typedef people_msgs::Person PP;

  TicTacToePPLSkill() {
    _last_ppl_was_set = false;
  }

private:
  virtual void create_subscribers_and_publishers_playzone() {
    std::string ppl_topic = "ppl";
    _nh_private.param("ppl_topic", ppl_topic, ppl_topic);
    _ppl_sub = _nh_private.subscribe(ppl_topic, 1, &TicTacToePPLSkill::ppl_cb, this);
    TicTacToeSkill::create_subscribers_and_publishers_playzone();
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void shutdown_subscribers_and_publishers_playzone() {
    _ppl_sub.shutdown();
    TicTacToeSkill::shutdown_subscribers_and_publishers_playzone();
  }

  //////////////////////////////////////////////////////////////////////////////

  void ppl_cb(const PPL::ConstPtr & ppl) {
    _last_ppl = *ppl;
    _last_ppl_was_set = true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! get the identity of the player
  virtual void first_move_hook(bool has_player_started) {
    // find the closest user
    if (_last_ppl_was_set) {
      bool ok = _player_tracker.set_user_to_nearest_person(_last_ppl);
    }
  }

  ros::Subscriber _ppl_sub;
  PPL _last_ppl;
  vision_utils::PPLUserTracker _player_tracker;
  bool _last_ppl_was_set;
};

#endif // TIC_TAC_TOE_PPL_SKILL_H
