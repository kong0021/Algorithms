# The CatsTrie class structure
class CatsTrie:
    """
    The constructor of CatsTrie class which is uses the data structure Trie to store or encapsulate all
    cat sentences from a set of strings from [a..z]. The Trie is created by iterating through N sentences
    to obtain M characters in the sentence and create new nodes representing them which can be identified by their data/key
    and the parent where M is the number of characters in the longest sentence. For sentences that occur more than once,
    its frequency will also be updated and stored as an attribute of the node. Each character represents a node and would
    have a link to their children node in respective order if the characters are in order and in the same sentence. The first
    character would be linked to the root node as the parent which does not represent any characters.

    Input:
        sentences: a list of strings with N sentences, where N is a positive integer and the longest sentence 
        would have M characters, as mapped from the cat vocabulary. M is a positive integer.

    Time complexity: O(NM) where N is the number of sentence in sentences and M is the number of characters in the longest sentence

    Aux space complexity: O(NM) where N is the number of sentence in sentences and M is the number of characters in the longest sentence
    """
    def __init__(self, sentences):

        # Initialize the root of trie
        self.root = Node("", None)

        # Initialize the root of trie
        self.root = Node("", None)

        # Iterate through sentences N times
        for sentence in sentences:
            currNode = self.root

            # Iterates through M characters in sentence
            for index in range(len(sentence)):
                
                # If the node's link is not None, assign the current node to its character 
                # The index represents the order of the characters from the first to last in the sentence
                if currNode.links[ord(sentence[index]) - 97] is not None:
                    currNode = currNode.links[ord(sentence[index]) - 97]
                # Else, assign the sentence to the node's data
                # Create a new node, with the current node as the parent node
                # Assign the link of the current node as the new node as well as assign current node as the new node
                else:
                    newData = currNode.data + sentence[index]
                    newNode = Node(newData, currNode)
                    currNode.links[ord(sentence[index]) - 97] = newNode
                    currNode = newNode
            # Frequency is incremented by 1 for every occurrence of the same sentence
            currNode.frequency += 1

            # update most frequency node (reversing M)
            compareNode = currNode
            for _ in range(len(sentence)):
                currMostFreqNode = currNode.mostFreqNode
                
                # most frequency node for the current node already exists
                if currMostFreqNode is not None:
                    # if the computed node (compareNode) frequency > existing one
                    if currMostFreqNode.frequency < compareNode.frequency:
                        currNode.mostFreqNode = compareNode
                        currNode = currNode.parent

                    # if the computed node (compareNode) frequency == existing one
                    elif currMostFreqNode.frequency == compareNode.frequency:
                        # we check lexicographical order 
                        # if the computed node (compareNode) smaller lexicographical order
                        if currMostFreqNode.data > compareNode.data:
                            currNode.mostFreqNode = compareNode
                        currNode = currNode.parent
                    
                    # if the computed node (compareNode) frequency < existing one
                    else:
                        break
                        
                # most frequency node for the current node does not exist
                else:
                    currNode.mostFreqNode = compareNode
                    currNode = currNode.parent

            # update root most frequency node (after for loop we will be at root node)
            # most frequency node for root does not exist
            if self.root.mostFreqNode is None:
                currNode.mostFreqNode = currNode
                
            # if the computed node (compareNode) frequency > most frequency node for root
            elif currNode.mostFreqNode.frequency < compareNode.frequency:
                currNode.mostFreqNode = compareNode

            # if the computed node (compareNode) frequency == most frequency node for root
            elif currNode.mostFreqNode.frequency == compareNode.frequency:
                # we check lexicographical order 
                # if the computed node (compareNode) smaller lexicographical order
                if currNode.mostFreqNode.data > compareNode.data:
                    currNode.mostFreqNode = compareNode

    """
    A function used to auto complete the sentence after a prompt is given. It takes the input
    prompt and checks the catsTrie data structure and auto completes the prompt into the inputted
    list of sentences based on the set of conditions stated in the catsTrie class which was based on the 
    highest frequency sentence.

    Input:
        prompt: prompt is a string with characters in the set of [a...z]. This string represents the
                incomplete sentence that is to be completed by the trie

    Output:
        returns a string that represents the completed sentence given the set of conditions in the 
        catsTrie class

    Time complexity: O(X+Y) where X is the length of prompt and Y is the length of the most 
                     frequent sentence in sentences.

    Aux space complexity: O(X) where X is the length of prompt

    """
    def autoComplete(self, prompt):
        currNode = self.root
        for char in prompt:
            if currNode.links[ord(char) - 97] is not None:
                currNode = currNode.links[ord(char) - 97]
            else:
                return None
        return currNode.mostFreqNode.data


class Node:
    """
    A constructor for node class which is created to represent the characters in every sentence.

    Input:
        data: The string representing the sentence
        parent: The parent node
    """
    def __init__(self, data, parent):
        self.data = data
        self.frequency = 0
        # a-z
        self.links = [None] * 26
        self.mostFreqNode = None
        self.parent = parent