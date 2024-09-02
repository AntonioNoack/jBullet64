
/***************************************************************************************************
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/
package com.bulletphysics.linearmath;

/**
 * Iterator to navigate through profile tree.
 * 
 * @author jezek2
 */
public class CProfileIterator {

	private CProfileNode currentParent;
	private CProfileNode currentChild;

	CProfileIterator(CProfileNode start) {
		currentParent = start;
		currentChild = currentParent.getChild();
	}
	
	// Access all the children of the current parent
	
	public void first() {
		currentChild = currentParent.getChild();
	}
	
	public void next() {
		currentChild = currentChild.getSibling();
	}
	
	public boolean isDone() {
		return (currentChild == null);
	}
	
	public boolean isRoot() {
		return (currentParent.getParent() == null);
	}

	/**
	 * Make the given child the new parent.
	 */
	public void enterChild(int index) {
		currentChild = currentParent.getChild();
		while ((currentChild != null) && (index != 0)) {
			index--;
			currentChild = currentChild.getSibling();
		}

		if (currentChild != null) {
			currentParent = currentChild;
			currentChild = currentParent.getChild();
		}
	}
	
	//public void enterLargestChild(); // Make the largest child the new parent
	
	/**
	 * Make the current parent's parent the new parent.
	 */
	public void enterParent() {
		if (currentParent.getParent() != null) {
			currentParent = currentParent.getParent();
		}
		currentChild = currentParent.getChild();
	}

	// Access the current child
	
	public String getCurrentName() {
		return currentChild.getName();
	}

	public int getCurrentTotalCalls() {
		return currentChild.getTotalCalls();
	}

	public double getCurrentTotalTime() {
		return currentChild.getTotalTime();
	}

	// Access the current parent
	
	public String getCurrentParentName() {
		return currentParent.getName();
	}

	public int getCurrentParentTotalCalls() {
		return currentParent.getTotalCalls();
	}

	public double getCurrentParentTotalTime() {
		return currentParent.getTotalTime();
	}
	
}
