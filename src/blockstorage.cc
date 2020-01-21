#include "blockstorage.h"
#include "common.h"

#include <cassert>
#include <iostream>

/**
 * @brief Initializes a local effect var_influence map, meaning every variable
 * is a product of just itself, e.g. <a, {a}>.
 *
 * @param in_scope_vars The in scope variables to include in the variable
 * influence map.
 * @return var_influence_map The neutral variable influence map of all variables
 * in scope.
 */
var_influence_map BlockStorage::initialize_local_effect(
    const std::vector<std::vector<decl_uid>> &in_scope_vars) {
  var_influence_map vim{};

  for (auto &var_scopes : in_scope_vars) {
    for (auto &in_scope_var : var_scopes) {
      vim.insert(std::pair<decl_uid, std::unordered_set<decl_uid>>(
          in_scope_var, {in_scope_var}));
    }
  }
  return vim;
}

/**
 * @brief Creates a new basic block starting with the id_tree statement. This
 * function should only be called upon verifying that the id_tree does not exist
 * in block_idx yet.
 *
 * @param id_tree The identifying statement of the block/the first statement in
 * the block. Can be a COND_EXPR.
 * @param in_scope_vars The variables that are in scope at the point of arriving
 * at the id_tree.
 * @return std::size_t The index of the newly created block.
 * Note 1: this function is basically making sure there is a single call to
 * create a node, which ensures the vectors stay in sync.
 */
std::size_t BlockStorage::initialize_basic_block(
    const tree id_tree,
    const std::vector<std::vector<decl_uid>> &in_scope_vars) {
  std::size_t new_block_idx = block_count++;
  block_idx.insert(std::pair<tree, std::size_t>(id_tree, new_block_idx));
  code_blocks.push_back({});
  local_effects.push_back({initialize_local_effect(in_scope_vars)});
  successors.push_back({});
  ref_idx_vars.push_back({});
  return new_block_idx;
}

/**
 * @brief Links a predecessor block by adding its successor to its successor
 * list.
 *
 * @param pre The predecessor index that should get a successor.
 * @param suc The successor to be added to the predecessors successors.
 * Note 1: this function is mainly kept in case adding predecessors would also
 * become feasible.
 */
void BlockStorage::link_blocks(const std::size_t pre, const std::size_t suc) {
  successors[pre].push_back(suc);
}

/**
 * @brief Returns the number of basic blocks in the BlockStorage.
 *
 * @return std::size_t The number of basic blocks in the BlockStorage.
 */
std::size_t BlockStorage::size() const { return block_count; }

/**
 * @brief Pushes stmt_tree to the statement vector of the block associated with
 * idx.
 *
 * @param idx The index of the block to push the stmt_tree for.
 * @param stmt_tree The stmt_tree to push.
 */
void BlockStorage::push_stmt(const std::size_t idx, const tree stmt_tree) {
  code_blocks[idx].push_back(stmt_tree);
}

/**
 * @brief Adds local effects for the in_scope_vars to the local effects of block
 * idx.
 *
 * @param idx The index of the block to add local effects for.
 * @param in_scope_vars The variables in scope used to describe local effects
 * for block idx.
 * Note 1: example usage: when entering a new scope.
 */
void BlockStorage::add_local_effect(
    const std::size_t idx, const std::vector<decl_uid> &in_scope_vars) {
  var_influence_map vim = initialize_local_effect({in_scope_vars});
  local_effects[idx].push_back(local_effects[idx].back());
  local_effects[idx].back().insert(vim.begin(), vim.end());
}

/**
 * @brief Removes local effects for the outdated vars in the local effects of
 * block idx.
 *
 * @param idx The index of the block to remove local effects for.
 * @param outdated_vars The variables going out of scope.
 * Note 1: example usage: when leaving a scope.
 */
void BlockStorage::remove_local_effects(
    const std::size_t idx, const std::vector<decl_uid> &outdated_vars) {
  local_effects[idx].push_back(local_effects[idx].back());
  for (auto &outdated_var : outdated_vars) {
    local_effects[idx].back().erase(outdated_var);
  }
}

/**
 * @brief Returns a successor for pre in the form of a block identified by
 * id_tree.
 *
 * @param pre The predecessor of the block identified by id_tree.
 * @param id_tree The tree identifying the requested block.
 * @param in_scope_vars The variables in scope whilst entering the block
 * identified by id_tree from pre.
 * @return std::size_t The index of the requested block.
 * Note 1: this should be the only function initializing basic blocks.
 * Note 2: checking for preexising blocks is really important for e.g. backwards
 * GOTO_EXPR.
 */
std::size_t BlockStorage::get_successor(
    const std::size_t pre, const tree id_tree,
    const std::vector<std::vector<decl_uid>> &in_scope_vars) {
  /* Pass the existing block or make a new one. Useful for GOTO_EXPR parsing. */
  std::size_t suc = contains(block_idx, id_tree)
                        ? block_idx.at(id_tree)
                        : initialize_basic_block(id_tree, in_scope_vars);
  link_blocks(pre, suc);
  return suc;
}

/**
 * @brief "Create" an unreachable basic block. This function is mainly for
 * completeness/clarity.
 *
 * @return std::size_t The index of the "new" unreachable block. This is
 * currently always the sink_idx and should be voided anyway for current
 * analysis goals.
 */
std::size_t BlockStorage::get_unreachable() {
  return get_successor(sink_idx, NULL_TREE, {{}});
}

/**
 * @brief Same as get_successor, but also saves the supplied binary comparison
 * vector for future analysis.
 *
 * @param pre The predecessor of the block identified by id_tree.
 * @param id_tree The tree identifying the requested block.
 * @param in_scope_vars The variables in scope whilst entering the block
 * identified by id_tree from pre.
 * @param bcv The binary comparison vector as extracted from the COND_EXPR
 * id_tree.
 * @return std::size_t The index of the requested block.
 * Note 1: The order of getting the successor first and then inserting the
 * comparisons is important.
 */
std::size_t BlockStorage::get_cond_successor(
    const std::size_t pre, const tree id_tree,
    const std::vector<std::vector<decl_uid>> &in_scope_vars,
    const bin_cmp_vec bcv) {
  /* order very important */
  std::size_t suc = get_successor(pre, id_tree, in_scope_vars);
  comparisons.insert({block_idx[id_tree], bcv});
  return suc;
}

/**
 * @brief Checks whether the block at index idx represents a COND_EXPR.
 *
 * @param idx The index to check for a COND_EXPR.
 * @return true The block corresponding to idx is identified by a COND_EXPR.
 * @return false The block corresponding to idx is not identified by a
 * COND_EXPR.
 */
bool BlockStorage::is_comparison(const std::size_t idx) const {
  return contains(comparisons, idx);
}

/**
 * @brief Adds the ref_incluences to the vector of ref tuples stored for block
 * idx.
 *
 * @param idx The index of the block the ref_influence is in.
 * @param ref_influence The reference index influence to process and add to the
 * storage vector (e.g. <arr[i+j], {i,j}>). Note 1: The ref_influence is
 * processed first: any variable influences stored so far in the block are used
 * to trace the variables actually influencing the reference index. Note 2: One
 * could also calculate in a later stage and save some storage space.
 */
void BlockStorage::add_ref(
    const std::size_t idx,
    const std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
        &ref_idx_influence) {
  assert(!local_effects.empty());
  ref_tuple resolved_reference_tuple;
  std::unordered_set<decl_uid> traced_idx_vars;
  var_influence_map latest_vim = local_effects[idx].back();
  std::unordered_set<decl_uid>::const_iterator it;
  for (std::size_t i = 0; i < ref_idx_influence.size(); ++i) {
    std::get<0>(resolved_reference_tuple) = ref_idx_influence[i].first;
    for (it = ref_idx_influence[i].second.begin();
         it != ref_idx_influence[i].second.end(); ++it) {
      if (contains(latest_vim, *it)) {
        traced_idx_vars.insert(latest_vim.at(*it).begin(),
                               latest_vim.at(*it).end());
      }
    }
    std::get<1>(resolved_reference_tuple) = traced_idx_vars;
    std::get<2>(resolved_reference_tuple) = local_effects[idx].size() - 1;
    ref_idx_vars[idx].push_back(resolved_reference_tuple);
    traced_idx_vars.clear();
  }
}

/**
 * @brief Pushes a new local effect based on inf and the latest local effect.
 *
 * @param idx The index of the block the variable influence takes place in.
 * @param inf The variable influence describing the change.
 */
void BlockStorage::update_effect(const std::size_t idx,
                                 const var_influence &inf) {
  std::unordered_set<decl_uid> collection;
  std::unordered_set<decl_uid>::const_iterator it;
  var_influence_map vim = local_effects[idx].back();
  std::unordered_set<decl_uid> match;
  for (it = inf.second.begin(); it != inf.second.end(); ++it) {
    if (contains(vim, *it)) {
      match = vim.at(*it);
      collection.insert(match.begin(), match.end());
    }
  }
  vim[inf.first] = collection;
  local_effects[idx].push_back(vim);
}

/**
 * @brief Construct a new Block Storage:: Block Storage object
 *
 * @param root_tree The statement the function in question starts with. Can be a
 * COND_EXPR.
 * @param parm_vars The PARM_DECL nodes associated with the function, which will
 * always be in scope. Note 1: The block initialization order matters because of
 * the root_idx and sink_idx choices.
 */
BlockStorage::BlockStorage(
    const tree root_tree, const std::vector<std::vector<decl_uid>> &parm_vars) {
  assert(root_tree != NULL_TREE);
  root_idx = 0;
  sink_idx = 1;
  block_count = 0;
  /* Order matters */
  initialize_basic_block(root_tree, parm_vars);
  initialize_basic_block(NULL_TREE, {{}});
}
