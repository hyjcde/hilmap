from ltlf2dfa.parser.ltlf import LTLfParser
import os
root_dir = r'/home/dyl/yzchen_ws/task_ws/hmi_misc/'
ltl_path = os.path.join(root_dir, 'output_ltl.txt')
buchi_path = os.path.join(root_dir, 'rqt_dfa.gv')

if __name__ == '__main__':
    parser = LTLfParser()

    formula_str = ''
    with open(ltl_path, "r") as fp:
        formula_str = fp.read()

    formula = parser(formula_str)
    dfa = formula.to_dfa()
    fp= open(buchi_path, "w")
    test_gv = fp.write(dfa)
    fp.close()
