#include "blockinterpreter.h"
#include "common.h"

#include <cassert>
#include <iostream>
#include <queue>

#include "diagnostic.h"

/**
 * @brief Adds a set of flagged variables to the state vector if it is not
 * present yet.
 *
 * @param idx The idx of the block the variables are flagged for.
 * @param flagged_vars The declaration uid's of the flagged variables.
 * @return true The variables have been added to the vector.
 * @return false The set of variables was already present in the vector.
 * Note 1: This function makes pushing to state_vec bahave like a set.
 * The (cleaner) alternative if defining hash methods for unordered sets of
 * decl_uids (ints).
 */
bool BlockInterpreter::add_state(std::size_t idx,
                                 std::unordered_set<decl_uid> flagged_vars) {
  int state_idx = find_in_vector(state_vec[idx], flagged_vars);
  if (state_idx >= 0) {
    return false;
  }
  state_vec[idx].push_back(flagged_vars);
  state_is_propagated[idx].push_back(false);
  return true;
}

/**
 * @brief Gathers all unpropagated states of for the idx block.
 *
 * @param idx The block to gather the unpropagated states for.
 * @return std::vector<std::unordered_set<decl_uid>> All unpropagated stated of
 * block idx.
 */
std::vector<std::unordered_set<decl_uid>>
BlockInterpreter::get_unpropagated_states(std::size_t idx) {
  std::vector<std::unordered_set<decl_uid>> unpropagated_states;

  for (std::size_t i = 0; i < state_is_propagated[idx].size(); ++i) {
    if (!state_is_propagated[idx][i]) {
      unpropagated_states.push_back(state_vec[idx][i]);
      state_is_propagated[idx][i] = true;
    }
  }
  return unpropagated_states;
}

/**
 * @brief Generates the set of variables that will be flagged by the local
 * effect of block idx with flagged_vars as input.
 *
 * @param idx The index of the block to calculate the outgoing state for.
 * @param flagged_vars The variables flagged before the block.
 * @param local_effect_idx The index of the local effect withing the vector of
 * local effects of block idx.
 * @return std::unordered_set<decl_uid> The set of decl_uid's flagged.
 */
std::unordered_set<decl_uid> BlockInterpreter::gen_outstate_at(
    std::size_t idx, const std::unordered_set<decl_uid> &flagged_vars,
    std::size_t local_effect_idx) {
  std::unordered_set<decl_uid> new_flags;
  var_influence_map local_effect = bs.local_effects[idx][local_effect_idx];
  std::unordered_set<decl_uid>::const_iterator flag_it;
  var_influence_map::const_iterator local_effect_it;
  for (flag_it = flagged_vars.begin(); flag_it != flagged_vars.end();
       ++flag_it) {
    for (local_effect_it = local_effect.begin();
         local_effect_it != local_effect.end(); ++local_effect_it) {
      if (contains(local_effect_it->second, *flag_it)) {
        new_flags.insert(local_effect_it->first);
      }
    }
  }
  return new_flags;
}

/**
 * @brief gen_outstate_at but for the last local effect available for the block.
 *
 * @param idx The index of the block to calculate the outgoing state for.
 * @param flagged_vars The variables flagged before the block.
 * @return std::unordered_set<decl_uid>
 * Note 1: Useful when blocks can be viewed as black boxes; use gen_outstate_at
 * when going over individual statements within thr block.
 */
std::unordered_set<decl_uid> BlockInterpreter::gen_outstate(
    std::size_t idx, const std::unordered_set<decl_uid> &flagged_vars) {
  assert(!bs.local_effects[idx].empty());
  return gen_outstate_at(idx, flagged_vars, bs.local_effects[idx].size() - 1);
}

/**
 * @brief Flags variables (second stage) whenever they appear on the opposite
 * side of flagged. Flagged variables are stores in cmp_flagged.
 *
 * @param cond_idx The index of the conditional block.
 * @param flagged The first stage flagged variables.
 */
void BlockInterpreter::add_cmp_flags(
    std::size_t cond_idx, const std::unordered_set<decl_uid> &flagged) {
  std::unordered_set<decl_uid> lhs, rhs;
  bin_cmp_vec bcv = bs.comparisons[cond_idx];
  for (auto &cmp : bcv) {
    lhs = unordered_intersection(flagged, cmp.first);
    rhs = unordered_intersection(flagged, cmp.second);
    if (!lhs.empty()) {
      /* If anything was flagged, add the pre-calced other side (NOT INTERSECT)
       */
      cmp_flagged[cond_idx].insert(cmp.second.begin(), cmp.second.end());
    }
    if (!rhs.empty()) {
      cmp_flagged[cond_idx].insert(cmp.first.begin(), cmp.first.end());
    }
  }
}

/**
 * @brief Generates an instance of the BlockStore, evaluated under the
 * flagged_vars.
 *
 * @param flagged_vars The variables that are initially flagged, upon which the
 * model instance will be based. Note: This function is meant to do the first
 * stage flagging, meaning flag spread by decl/modify.
 */
void BlockInterpreter::gen_bs_instance(
    const std::unordered_set<decl_uid> &flagged_vars) {
  std::size_t idx = bs.root_idx;
  std::unordered_set<std::size_t> updated_suc;
  std::queue<std::size_t> block_queue;
  std::vector<std::unordered_set<decl_uid>> unpropagated_states;
  std::unordered_set<decl_uid> result_state;

  result_state = gen_outstate(idx, flagged_vars);
  add_state(idx, result_state);
  block_queue.push(idx);

  while (!block_queue.empty()) {
    idx = block_queue.front();
    block_queue.pop();

    unpropagated_states = get_unpropagated_states(idx);

    for (auto &unpropagated_state : unpropagated_states) {
      for (auto &suc : bs.successors[idx]) {

        if (bs.is_comparison(suc)) {
          /* add flagged vars according to the cond's comparison */
          add_cmp_flags(suc, unpropagated_state);
        }

        result_state = gen_outstate(suc, unpropagated_state);
        if (add_state(suc, result_state)) {
          updated_suc.insert(suc);
        }
      }
    }

    /* Update the nodes that might need checking */
    for (const auto &i : updated_suc) {
      block_queue.push(i);
    }
    updated_suc.clear();
  }
}

/**
 * @brief Extends the instance generated by gen_bs_instance by generating second
 * stage flags.
 *
 * Note: This function is meant to do the second stage flagging, meaning flag
 * spread by relational comparison with a precision difference.
 */
void BlockInterpreter::gen_cmp_flagged_instance() {
  std::size_t idx = bs.root_idx;
  std::queue<std::size_t> block_queue;
  std::unordered_set<std::size_t> done;
  std::unordered_set<decl_uid> outstate;

  block_queue.push(idx);
  done.insert(idx);

  /* Simple one time visit BFS */
  while (!block_queue.empty()) {
    idx = block_queue.front();
    block_queue.pop();

    for (auto &suc : bs.successors[idx]) {

      /* Generate the next full states */
      outstate = gen_outstate(suc, cmp_flagged[idx]);
      cmp_flagged[suc].insert(outstate.begin(), outstate.end());

      if (!contains(done, suc)) {
        block_queue.push(suc);
        done.insert(suc);
      }
    }
  }
}

/**
 * @brief Reports a suspected loop counter variable range mismatch, mentioning
 * location and function name.
 *
 * @param reference The reference using a potentially dangerous loop variable.
 * @param func_name The name of the function currently being inspected.
 */
void BlockInterpreter::report(const tree reference) {
  opt_code trigger_flag = OPT_Wall;
  const char * msg = "Loop ctr var suspected range mismatch.";
  if(EXPR_HAS_LOCATION(reference)){
    warning_at(EXPR_LOCATION(reference), trigger_flag, msg);
  }
  else{
    warning(trigger_flag, msg);
  }
}

/**
 * @brief Checks the instance generated by gen_bs_instance and
 * gen_cmp_flagged_instance; by inspecting reference indices for second stage
 * flagged variables.
 *
 * @param func_name The name of the function currently being inspected.
 */
void BlockInterpreter::check_cmp_flagged_instance() {
  std::unordered_set<decl_uid> ref_state;
  std::unordered_set<decl_uid> intersection;
  std::unordered_set<tree> final_suspects;

  for (std::size_t i = 0; i < bs.block_idx.size(); ++i) {
    for (auto &suc : bs.successors[i]) {
      for (auto &ref : bs.ref_idx_vars[suc]) {
        if (contains(final_suspects, std::get<0>(ref))) {
          break;
        }

        // The variables flagged in idx, checked under the successors _th update
        ref_state = gen_outstate_at(suc, cmp_flagged[i], std::get<2>(ref));
        intersection = unordered_intersection(std::get<1>(ref), ref_state);

        if (!intersection.empty()) {
          final_suspects.insert(std::get<0>(ref));
          report(std::get<0>(ref));
        }
      }
    }
  }
}

/**
 * @brief Convenient grouped call for creating a model instance and checking it
 * for the bug.
 *
 * @param flagged_vars The variables that are initially flagged, upon which the
 * model instance will be based.
 * @param func_name The name of the function currently being inspected.
 */
void BlockInterpreter::gen_and_check_model(
    const std::unordered_set<decl_uid> &flagged_vars) {
  gen_bs_instance(flagged_vars);
  gen_cmp_flagged_instance();
  check_cmp_flagged_instance();
}

/**
 * @brief Construct a new Block Interpreter:: Block Interpreter object
 *
 * @param bs The BlockStore to create the instance out of / to interprete.
 */
BlockInterpreter::BlockInterpreter(BlockStorage bs) : bs(bs) {
  cmp_flagged = std::vector<std::unordered_set<decl_uid>>(
      bs.block_idx.size(), std::unordered_set<decl_uid>());
  state_vec = std::vector<std::vector<std::unordered_set<decl_uid>>>(
      bs.block_idx.size(), std::vector<std::unordered_set<decl_uid>>());
  state_is_propagated =
      std::vector<std::vector<bool>>(state_vec.size(), std::vector<bool>());
}
