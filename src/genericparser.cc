#include "genericparser.h"
#include "common.h"

#include <cassert>
#include <iostream>

#include "print-tree.h"
#include "tree-iterator.h"

/**
 * @brief Sets specifying what to redo and keep in function
 * GenericParser::categorize_operand.
 *
 */
struct GenericParser::expr_filter {
  std::unordered_set<tree_code> redo;
  std::unordered_set<tree_code> keep;
};

/**
 * @brief Basic composite expression set.
 * These are not of interest, but their operands could be (redo).
 * Note 1: INDIRECT_REF, ARRAY_REF, and ADDR_EXPR may NEVER be added.
 * Note 2: The NOP_EXPR is important, as it is used for conversions etc.
 * Note 3: Anything not in here will not be parsed further whilst processing
 * DECL_EXPR and MODIFY_EXPR, including reference indices.
 */
const std::unordered_set<tree_code> GenericParser::basic_composite_expr{
    NOP_EXPR,  CONVERT_EXPR, PLUS_EXPR,        MINUS_EXPR,
    MULT_EXPR, CONVERT_EXPR, POINTER_PLUS_EXPR};

/**
 * @brief Relational operators set.
 * The operators and (nested) variables are of interest when examining
 * conditional nodes. Note 1: EQ_EXPR and NE_EXPR are left out.
 */
const std::unordered_set<tree_code> GenericParser::relational_operators{
    LT_EXPR, LE_EXPR, GT_EXPR, GE_EXPR};

/**
 * @brief Logical operators set.
 * The variables can be of interest.
 */
const std::unordered_set<tree_code> GenericParser::logical_operators{
    TRUTH_ANDIF_EXPR, TRUTH_ORIF_EXPR, TRUTH_AND_EXPR,
    TRUTH_OR_EXPR,    TRUTH_XOR_EXPR,  TRUTH_NOT_EXPR};

/**
 * @brief The expression filter collecting relational operators in conditional
 * expressions.
 *
 */
const GenericParser::expr_filter GenericParser::cond_expr_filter{
    unordered_union(basic_composite_expr, logical_operators), // redo
    relational_operators                                      // keep
};

/**
 * @brief Pushes all statements from a STATMENT_LIST on the stack.
 *
 * @param stmt_list The STATEMENT_LIST to put on the stack.
 */
void GenericParser::push_stmt_list_on_stack(const tree stmt_list) {
  assert(TREE_CODE(stmt_list) == STATEMENT_LIST);
  tree_stmt_iterator head = tsi_start(stmt_list);
  if (tsi_end_p(head)) {
    return;
  }
  for (tree_stmt_iterator t = tsi_last(stmt_list);
       tsi_stmt_ptr(t) != tsi_stmt_ptr(head); tsi_prev(&t)) {
    stmt_stack.push(tsi_stmt(t));
  }
  stmt_stack.push(tsi_stmt(head));
}

/**
 * @brief Converts a chain of [VAR/LABEL]_TREE to a vector of VAR_TREE
 * decl_uid's.
 *
 * @param decl the start of the declaration chain
 * @return std::vector<decl_uid> decl uid's of all VAR_DECL trees chained from
 * decl
 */
std::vector<decl_uid> GenericParser::decl_tree_to_uids(const tree decl) {
  if (decl == NULL_TREE) {
    return {};
  }
  assert(DECL_P(decl));
  /* {FIELD_DECL, VAR_DECL, CONST_DECL, PARM_DECL, TYPE_DECL, RESULT_DECL, etc} */
  std::vector<decl_uid> variables;
  tree t = decl;
  while (t) {
    /* Label decl check */
    if (TREE_CODE(t) == VAR_DECL) {
      variables.push_back(DECL_UID(t));
    }
    t = TREE_CHAIN(t);
  }
  return variables;
}

/**
 * @brief Splits a BIND_EXPR and stores it into *vars and *body.
 *
 * @param bind_expr The BIND_EXPR to split.
 * @param vars OUT: Pointer to save the bind expression variables to.
 * @param body OUT: Pointer to save the bind expression body to.
 */
void GenericParser::split_bind_expr(const tree bind_expr, tree *const vars,
                                     tree *const body) {
  assert(TREE_CODE(bind_expr) == BIND_EXPR);
  *vars = BIND_EXPR_VARS(bind_expr);
  *body = BIND_EXPR_BODY(bind_expr);
}

/**
 * @brief Puts a single operand in the appropriate container according to
 * filter.
 *
 * @param operand The operand to categorize.
 * @param filter The filter.
 * @param redo IN&OUT: the redo/compound statement stack.
 * @param keep IN&OUT: the keep vector.
 */
void GenericParser::categorize_operand(const tree operand,
                                        const expr_filter &filter,
                                        std::stack<tree> &redo,
                                        std::vector<tree> &keep) {
  tree_code op_tree_code = TREE_CODE(operand);
  if (contains(filter.keep, op_tree_code)) {
    keep.push_back(operand);
  } else if (contains(filter.redo, op_tree_code)) {
    redo.push(operand);
  } else {
    /* Trees not processed further */
  }
  return; // Throw away, e.g. for CST
}

/**
 * @brief Flattens the root expression tree according to filter.
 *
 * @param root_expr the root expression tree
 * @param filter the filter
 * @return std::vector<tree> The expressions the filter deems keepworthy.
 * Note 1: A call_expr can be argued to be too complex to treat this way,
 * Hence why it is skipped when found.
 */
std::vector<tree> GenericParser::flatten_expr(const tree root_expr,
                                               const expr_filter &filter) {
  std::vector<tree> expr_vector;
  /* Base case: root_expr is in the keep set. */
  if (contains(filter.keep, TREE_CODE(root_expr))) {
    expr_vector.push_back(root_expr);
    return expr_vector;
  }
  std::stack<tree> composite_expr_stack;
  composite_expr_stack.push(root_expr);
  tree operand, t;
  int i;
  while (!composite_expr_stack.empty()) {
    t = composite_expr_stack.top();
    composite_expr_stack.pop();
    if (TREE_CODE(t) == CALL_EXPR) {
      continue;
    }
    for (i = 0; i < TREE_OPERAND_LENGTH(t); ++i) {
      operand = TREE_OPERAND(t, i);
      if (operand == NULL_TREE) {
        continue; 
      }
      categorize_operand(operand, filter, composite_expr_stack, expr_vector);
    }
  }
  return expr_vector;
}

/**
 * @brief Splits decl_expr and stores it into *lvalue and *rvalue.
 *
 * @param decl_expr The declaration expression to split.
 * @param lvalue OUT: The lvalue the decl is stored in.
 * @param rvalue OUT: The rvalue the initial value is stored in.
 * @return true The initial value (rvalue) is a null tree.
 * @return false The initial value (rvalue) is not a null tree.
 */
bool GenericParser::split_decl_expr(const tree decl_expr, tree *const lvalue,
                                     tree *const rvalue) {
  assert(TREE_CODE(decl_expr) == DECL_EXPR);
  *lvalue = DECL_EXPR_DECL(decl_expr);
  *rvalue = DECL_INITIAL(*lvalue);
  return *rvalue == NULL_TREE;
}

/**
 * @brief Splits modify_expr and stores it into *lvalue and *rvalue.
 *
 * @param modify_expr The modify_expr to split.
 * @param lvalue OUT: the lvalue.
 * @param rvalue OUT: the rvalue.
 */
void GenericParser::split_modify_expr(const tree modify_expr,
                                       tree *const lvalue, tree *const rvalue) {
  assert(TREE_CODE(modify_expr) == MODIFY_EXPR);
  *lvalue = TREE_OPERAND(modify_expr, 0);
  *rvalue = TREE_OPERAND(modify_expr, 1);
}

/**
 * @brief Flattens an expression into a var_influence.
 *
 * @param lhs The lhs of the expression, the variable that is changed.
 * @param rhs The rhs of the expression, the expression describing the new value
 * of lhs.
 * @return var_influence The variable influence described by the input
 * expression, consisting of VAR_DECL and PARM_DECL trees.
 */
var_influence GenericParser::detach_expr(const tree lhs, const tree rhs) {
  std::vector<decl_uid> dus;
  std::vector<tree> rhs_vars =
      flatten_expr(rhs, {basic_composite_expr, {VAR_DECL, PARM_DECL}});

  for (const auto &rhs_var : rhs_vars) {
    dus.push_back(DECL_UID(rhs_var));
  }

  return var_influence(
      DECL_UID(lhs),
      std::unordered_set<decl_uid>(dus.begin(), dus.end()) 
  );
}

/**
 * @brief Calculated the maximum precision over a vector of variable trees.
 *
 * @param vars The variable trees.
 * @return int The maximum precision found in vars.
 * Note 1: the unsigned subtraction makes this work for signed/unsigned
 * differences.
 */
int GenericParser::max_precision(const std::vector<tree> &vars) {
  int max_var_precision = 0;
  int var_precision = 0;
  for (auto &var : vars) {
    if (TREE_CODE(TREE_TYPE(var)) == INTEGER_TYPE) {
      var_precision =
          TYPE_PRECISION(TREE_TYPE(var)) - !TYPE_UNSIGNED(TREE_TYPE(var));
      if (var_precision > max_var_precision) {
        max_var_precision = var_precision;
      }
    }
  }
  return max_var_precision;
}

/**
 * @brief Checks whether the operator has a possibly alarming presicion
 * difference.
 *
 * @param relational_operator_code The tree code of the relational operator.
 * @param lhs_vars The tree on the lhs of the relational operator.
 * @param rhs_vars The tree on the rhs of the relational operator.
 * @return true The precision difference is potentially alarming for this
 * operator.
 * @return false The precision difference is not alarming for this operator.
 * Note: rationale: a less-than expression with a loop counter variable
 * counting upwards can loop around if its precision is less than the bound.
 * Similar when counting down and doing a lower bound check.
 */
bool GenericParser::has_precision_diff(tree_code relational_operator_code,
                                        const std::vector<tree> &lhs_vars,
                                        const std::vector<tree> &rhs_vars) {
  int max_lhs_precision = max_precision(lhs_vars);
  int max_rhs_precision = max_precision(rhs_vars);
  switch (relational_operator_code) {
  case LT_EXPR:
  case LE_EXPR:
    return max_lhs_precision < max_rhs_precision;
  case GT_EXPR:
  case GE_EXPR:
    return max_lhs_precision > max_rhs_precision;
  default:
    break;
  }
  return false;
}

/**
 * @brief Splits the cond into a binary comparison vector.
 *
 * @param cond The conditon tree to be split.
 * @return bin_cmp_vec The individual binary comparisons, which are only
 * relational operators.
 */
bin_cmp_vec GenericParser::detach_condition(const tree cond) {
  bin_cmp_vec bcv;
  std::vector<tree> bin_cmps_unflattenend =
      flatten_expr(cond, cond_expr_filter);
  std::vector<tree> lhs_vars, rhs_vars;
  bin_cmp temp{};
  for (const auto &unflattened_bin_cmp : bin_cmps_unflattenend) {
    lhs_vars = flatten_expr(TREE_OPERAND(unflattened_bin_cmp, 0),
                            {basic_composite_expr, {VAR_DECL, PARM_DECL}});
    rhs_vars = flatten_expr(TREE_OPERAND(unflattened_bin_cmp, 1),
                            {basic_composite_expr, {VAR_DECL, PARM_DECL}});
    if (!has_precision_diff(TREE_CODE(unflattened_bin_cmp), lhs_vars,
                            rhs_vars)) {
      continue;
    }
    if (lhs_vars.size() != 0 && rhs_vars.size() != 0) {
      for (const auto &lhs_var : lhs_vars) {
        temp.first.insert(DECL_UID(lhs_var));
      }
      for (const auto &rhs_var : rhs_vars) {
        temp.second.insert(DECL_UID(rhs_var));
      }
      bcv.push_back(temp);
    }
  }
  return bcv;
}

/**
 * @brief Returns the index used in the reference.
 *
 * @param ref The reference to get the index tree from.
 * @return tree The index tree used in the reference.
 */
tree GenericParser::ref_index_tree(const tree ref) {
  tree retval;
  switch (TREE_CODE(ref)) {
  case ARRAY_REF:
    retval = TREE_OPERAND(ref, 1);
    break;
  case INDIRECT_REF:
    retval = TREE_OPERAND(ref, 0);
    break;
  default:
    throw std::invalid_argument("Not a ref tree.");
  }
  return retval;
}

/**
 * @brief Picks out the references in expr and returns them as <ref tree, index
 * variables> pairs.
 *
 * @param expr The expr to find the references in.
 * @return std::vector<std::pair<tree, std::unordered_set<decl_uid>>> The
 * reference trees and the variables used in their indices. Note 1: It is very
 * important to note that the second flatten_expr call ignores any ADDR_EXPR,
 * which makes this function also work for INDIRECT_REF instead of just
 * ARRAY_REF.
 */
std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
GenericParser::detach_references(const tree expr) {
  std::vector<tree> references =
      flatten_expr(expr, {basic_composite_expr, {ARRAY_REF, INDIRECT_REF}});
  std::vector<tree> idx_vars;
  std::unordered_set<decl_uid> idx_vars_uid;
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>> tree_idx_pairs;

  for (const auto &ref : references) {
    idx_vars = flatten_expr(ref_index_tree(ref),
                            {basic_composite_expr, {VAR_DECL, PARM_DECL}});
    for (const auto &idx_var : idx_vars) {
      idx_vars_uid.insert(DECL_UID(idx_var));
    }
    tree_idx_pairs.push_back(
        std::pair<tree, std::unordered_set<decl_uid>>(ref, idx_vars_uid));
  }
  return tree_idx_pairs;
}

/**
 * @brief Picks out the references in a MODIFY_EXPR, which can occur on both
 * sides.
 *
 * @param modify_expr The MODIFY_EXPR to get the references from.
 * @return std::vector<std::pair<tree, std::unordered_set<decl_uid>>> The
 * reference trees and the variables used in their indices.
 */
std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
GenericParser::detach_modify_expr_refs(const tree modify_expr) {
  tree lhs, rhs;
  split_modify_expr(modify_expr, &lhs, &rhs);

  std::vector<std::pair<tree, std::unordered_set<decl_uid>>> lhs_ref =
      detach_references(lhs);
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>> ref_collection =
      detach_references(rhs);
  ref_collection.insert(std::end(ref_collection), std::begin(lhs_ref),
                        std::end(lhs_ref));
  return ref_collection;
}

/**
 * @brief Splits a DECL_EXPR and returns its references; empty if the initial
 * value is a NULL_TREE.
 *
 * @param decl_expr The DECL_EXPR to get the references from.
 * @return std::vector<std::pair<tree, std::unordered_set<decl_uid>>> The
 * reference trees and the variables used in their indices.
 */
std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
GenericParser::detach_decl_expr_refs(const tree decl_expr) {
  assert(TREE_CODE(decl_expr) == DECL_EXPR);
  tree lhs, rhs;
  return split_decl_expr(decl_expr, &lhs, &rhs)
             ? std::vector<std::pair<tree, std::unordered_set<decl_uid>>>()
             : detach_references(rhs);
}

/**
 * @brief Unified call for detaching DECL_EXPR or MODIFY_EXPR
 *
 * @param t The DECL_EXPR or MODIFY_EXPR to get the references from.
 * @return std::vector<std::pair<tree, std::unordered_set<decl_uid>>> The
 * reference trees and the variables used in their indices.
 */
std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
GenericParser::detach_decl_or_modify_refs(const tree t) {
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>> retval;
  switch (TREE_CODE(t)) {
  case DECL_EXPR:
    retval = detach_decl_expr_refs(t);
    break;
  case MODIFY_EXPR:
    retval = detach_modify_expr_refs(t);
    break;
  default:
    throw std::invalid_argument("Not a decl/modify tree.");
  }
  return retval;
}

/**
 * @brief Created a var influence for the given MODIFY_EXPR.
 *
 * @param modify_expr The MODIFY_EXPR to create the var_influence from.
 * @return var_influence The var_influence derives from modify_expr. Empty if
 * the lhs is not recognized as a VAR_DECL or PARM_DECL. Note 1: This also means
 * this does not support references.
 */
var_influence GenericParser::detach_modify_expr(const tree modify_expr) {
  assert(TREE_CODE(modify_expr) == MODIFY_EXPR);
  tree lhs, rhs;
  var_influence retval;
  split_modify_expr(modify_expr, &lhs, &rhs);
  if (TREE_CODE(lhs) != VAR_DECL && TREE_CODE(lhs) != PARM_DECL) {
    retval = var_influence();
  } else {
    retval = detach_expr(lhs, rhs);
  }
  return retval;
}

/**
 * @brief Created a var influence for the given DECL_EXPR.
 *
 * @param decl_expr The DECL_EXPR to create the var_influence from.
 * @return var_influence The var_influence derives from decl_expr. Empty if the
 * rvalue is a NULL_TREE.
 */
var_influence GenericParser::detach_decl_expr(const tree decl_expr) {
  assert(TREE_CODE(decl_expr) == DECL_EXPR);
  tree lhs, rhs;
  // return something if the rvalue is not a null tree
  return split_decl_expr(decl_expr, &lhs, &rhs) ? var_influence()
                                                : detach_expr(lhs, rhs);
}

/**
 * @brief Unified call for creating a var influence from a DECL_EXPR or a
 * MODIFY_EXPR
 *
 * @param t The DECL_EXPR or MODIFY_EXPR to create the var influence from.
 * @return var_influence the var influence created from t.
 */
var_influence GenericParser::detach_decl_or_modify(const tree t) {
  assert(TREE_CODE(t) == DECL_EXPR || TREE_CODE(t) == MODIFY_EXPR);
  var_influence retval;
  switch (TREE_CODE(t)) {
  case DECL_EXPR:
    retval = detach_decl_expr(t);
    break;
  case MODIFY_EXPR:
    retval = detach_modify_expr(t);
    break;
  default:
    throw std::invalid_argument("Not a decl/modify tree.");
  }
  return retval;
}

/**
 * @brief Splits a conditional expression into its then-value and its else
 * value.
 *
 * @param cond_expr The COND_EXPR to get the then-value tree and the else-value
 * tree (NULL_TREE possible) from.
 * @param then_value OUT: The then-value tree.
 * @param else_value OUT: The else-value tree.
 * Note 1: important to check the else-value for NULL_TREE.
 */
void GenericParser::split_cond_expr(const tree cond_expr,
                                     tree *const then_value,
                                     tree *const else_value) {
  assert(TREE_CODE(cond_expr) == COND_EXPR);
  *then_value = COND_EXPR_THEN(cond_expr);
  /* Always check this value for a NULL_TREE: */
  *else_value = COND_EXPR_ELSE(cond_expr);
}

/**
 * @brief Peek the first non-[STATEMENT_LIST/BIND_EXRP] stmt in branch.
 *
 * @param branch The branch to peek the first statement for.
 * @return tree The first non-[STATEMENT_LIST/BIND_EXRP] stmt in branch.
 */
tree GenericParser::peek_branch_head(const tree branch) {
  assert(branch != NULL_TREE);
  tree head_stmt = branch;
  tree_code tc = TREE_CODE(head_stmt);

  while (tc == STATEMENT_LIST || tc == BIND_EXPR) {
    switch (tc) {
    case STATEMENT_LIST:
      head_stmt = tsi_stmt(tsi_start(head_stmt));
      break;
    case BIND_EXPR:
      head_stmt = BIND_EXPR_BODY(head_stmt);
      break;
    default: // do nothing
      break;
    }
    assert(head_stmt != NULL_TREE);
    tc = TREE_CODE(head_stmt);
  }
  return head_stmt;
}

// peek branch tail/head are not symmetric due to the way I chose to represent
// COND_EXPR.

/**
 * @brief Peek the last non-[STATEMENT_LIST/BIND_EXRP/COND_EXPR] stmt in branch.
 *
 * @param branch The branch to peek the last statement for.
 * @return tree The first non-[STATEMENT_LIST/BIND_EXRP] stmt in branch.
 * Note 1: peek_branch_head and peek_branch_tail are not symmetric (or: totally
 * similar), due to the way COND_EXPR are represented in blockstorage.h.
 */
tree GenericParser::peek_branch_tail(const tree branch) {
  assert(branch != NULL_TREE);
  tree tail_stmt, temp;
  tail_stmt = branch;
  tree_code tc = TREE_CODE(tail_stmt);

  while (tc == STATEMENT_LIST || tc == COND_EXPR || tc == BIND_EXPR) {
    switch (tc) {
    case STATEMENT_LIST:
      tail_stmt = tsi_stmt(tsi_last(tail_stmt));
      break;
    case COND_EXPR:
      temp = COND_EXPR_ELSE(tail_stmt);
      if (temp == NULL_TREE) {
        tail_stmt = COND_EXPR_THEN(tail_stmt);
      } else {
        tail_stmt = temp;
      }
      break;
    case BIND_EXPR:
      tail_stmt = BIND_EXPR_BODY(tail_stmt);
      break;
    default: // do nothing
      break;
    }
    assert(tail_stmt != NULL_TREE);
    tc = TREE_CODE(tail_stmt);
  }
  return tail_stmt;
}

/**
 * @brief Pushes the conditonal statement checkpoints in cond_expr, arrived at
 * from node idx, onto the conditional stack.
 *
 * @param cond_expr The CONDITIONAL_EXPR to put on the conditional stack.
 * @param idx The index of the block that led to cond_expr.
 * Note 1: A conditonal expression follows a rhombus shaped graph,
 * for which this conditional stack entry should provide all block/node entry
 * points.
 */
void GenericParser::push_cond_stmt_checkpoints(const tree cond_expr,
                                                const std::size_t idx) {
  assert(TREE_CODE(cond_expr) == COND_EXPR);
  tree then_value, else_value, after;
  after = stmt_stack.empty() ? NULL_TREE : peek_branch_head(stmt_stack.top());
  split_cond_expr(cond_expr, &then_value, &else_value);
  /* If there is no else value, we look for the after tree which is already on
   * the stack. */
  if (else_value != NULL_TREE) {
    stmt_stack.push(else_value);
  }
  stmt_stack.push(then_value); 
  then_value = peek_branch_head(then_value);
  else_value = (else_value == NULL_TREE) ? after : peek_branch_head(else_value);
  cond_stmt_checkpoints.push(cond_tuple(then_value, else_value, after, idx));
}

/**
 * @brief Puts the bind_expr body (statement list tree) on the stack.
 *
 * @param bind_expr The bind expr to put variables/body onto the stack for.
 */
void GenericParser::push_bind_expr_on_stack(const tree bind_expr) {
  assert(TREE_CODE(bind_expr) == BIND_EXPR);
  tree vars, body;
  std::vector<decl_uid> var_decls;
  split_bind_expr(bind_expr, &vars, &body);
  assert(body != NULL_TREE);

  if (TREE_CODE(body) == STATEMENT_LIST) {
    push_stmt_list_on_stack(body);
  } else {
    stmt_stack.push(body);
  }

  if (vars != NULL_TREE) {
    var_decls = decl_tree_to_uids(vars);
    if (!var_decls.empty()) {
      scope_end_checkpoints.push(peek_branch_tail(body));
      active_variables.push_back(var_decls);
    }
  }
}

/**
 * @brief Puts PARSM_DECL's from a FUNCTION_DECL onto the active variable stack.
 *
 * @param function_decl The FUNCTION_DECL to put the PARM_DECL's onto the stack
 * for. Note 1: This works a little differnt from a BIND_EXPR, therfore it needs
 * its own function.
 */
void GenericParser::parm_decl_to_stack(const tree function_decl) {
  std::vector<decl_uid> parm_decls;
  tree decl_argument = DECL_ARGUMENTS(function_decl);
  while (decl_argument) {
    parm_decls.push_back(DECL_UID(decl_argument));
    decl_argument = TREE_CHAIN(decl_argument);
  }
  active_variables.push_back(parm_decls);
}

/**
 * @brief Parses a FUNCTION_DECL into the blockstorage.h format.
 *
 * @param function_decl The FUNCTION_DECL to parse into a BlockStorage.
 * @return BlockStorage The BlockStorage corresponding to the function_decl.
 */
BlockStorage GenericParser::traverse_stmt_tree(const tree function_decl) {
  /* General parsing data */
  tree root = DECL_SAVED_TREE(function_decl);

  /* A function has a varless BIND_EXPR, but still posesses a scope.  */
  parm_decl_to_stack(function_decl);
  scope_end_checkpoints.push(peek_branch_tail(BIND_EXPR_BODY(root)));

  /* Returned BlockStorage */
  BlockStorage bs(peek_branch_head(root),
                active_variables); 

  /* Temporaries */
  std::size_t idx = bs.root_idx; 
  std::pair<tree, std::vector<decl_uid>> new_scope;
  cond_tuple top;
  bin_cmp_vec comparisons;
  var_influence vi;
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>> ref_idx;

  stmt_stack.push(root);
  tree t;
  while (!stmt_stack.empty()) {
    t = stmt_stack.top();
    stmt_stack.pop();
    assert(t != NULL_TREE);
    assert(scope_end_checkpoints.size() == active_variables.size());

    while (!cond_stmt_checkpoints.empty()) {
      top = cond_stmt_checkpoints.top();
      /* Not elegant because C++ constant only switch*/
      if (t == std::get<0>(top)) {
        /* the successor of from block is idx , the then branch */
        idx = bs.get_successor(std::get<3>(top), t,
                               active_variables); 
        break; // the if statement is never the else / after
      } else if (t == std::get<1>(top)) {
        /* The successor of the then block is the after tree */
        bs.get_successor(idx, std::get<2>(top), active_variables);
        /* the successor of from block is idx, the else branch */
        idx = bs.get_successor(std::get<3>(top), t, active_variables);
        /* If the else branch does not exist (= after branch), pop. */
        if (t == std::get<2>(top)) {
          cond_stmt_checkpoints.pop();
        } else
          break; 
      } else if (t == std::get<2>(top)) {
        /* the successor of from block is the after tree */
        idx = bs.get_successor(idx, std::get<2>(top), active_variables);
        cond_stmt_checkpoints.pop(); 
      } else
        break;
    }

    switch (TREE_CODE(t)) {
    case STATEMENT_LIST:
      push_stmt_list_on_stack(t);
      break;
    case BIND_EXPR:
      push_bind_expr_on_stack(t);
      bs.add_local_effect(idx, new_scope.second);
      break;
    case COND_EXPR:
      comparisons = detach_condition(COND_EXPR_COND(t));
      idx = bs.get_cond_successor(idx, t, active_variables, comparisons);
      push_cond_stmt_checkpoints(t, idx);
      break;
    case GOTO_EXPR:
      bs.get_successor(idx, GOTO_DESTINATION(t),
                       active_variables);
      idx = bs.get_unreachable();
      break;
    case LABEL_EXPR:
      idx = bs.get_successor(idx, LABEL_EXPR_LABEL(t), active_variables);
      break;
    case MODIFY_EXPR:
    case DECL_EXPR:
      vi = detach_decl_or_modify(t);
      ref_idx = detach_decl_or_modify_refs(t);
      // the ordering is important here
      if (!ref_idx.empty()) {
        bs.add_ref(idx, ref_idx);
      }
      if (vi.first) {
        bs.update_effect(idx, vi);
      }
      bs.push_stmt(idx, t);
      break;
    default:
      bs.push_stmt(idx, t);
      break;
    }
    /* While: multiple scopes may end after the same statement */
    while (!scope_end_checkpoints.empty() && t == scope_end_checkpoints.top()) {
      bs.remove_local_effects(idx, active_variables.back());
      active_variables.pop_back();
      scope_end_checkpoints.pop();
    }
  }
  assert(active_variables.empty());
  assert(scope_end_checkpoints.empty());
  return bs;
}

/**
 * @brief Checks a FUNCTION_DECL for integer parameters with a precision bigger
 * than min_long_int_precision.
 *
 * @param function_decl The FUNCTION_DECL to check for integer parameters with a
 * precision bigger than min_long_int_precision.
 * @param min_long_int_precision The minimum precision classifying an integer as
 * longer than normal.
 * @return std::unordered_set<decl_uid> The set of decl uid's of the integers
 * classified as longer than normal.
 */
std::unordered_set<decl_uid>
GenericParser::flag_decl_args(const tree function_decl,
                               const unsigned int min_long_int_precision) {
  decl_uid du;
  std::unordered_set<decl_uid> flagged;
  tree decl_argument = DECL_ARGUMENTS(function_decl);
  while (decl_argument) {
    if (TREE_CODE(decl_argument) == PARM_DECL &&
        TREE_CODE(TREE_TYPE(decl_argument)) == INTEGER_TYPE) {
      if (TYPE_PRECISION(TREE_TYPE(decl_argument)) >= min_long_int_precision) {
        du = DECL_UID(decl_argument);
        flagged.insert(du);
      }
    }
    decl_argument = TREE_CHAIN(decl_argument);
  }
  return flagged;
}

GenericParser::GenericParser(){};
