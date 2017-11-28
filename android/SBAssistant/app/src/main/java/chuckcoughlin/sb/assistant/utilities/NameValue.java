/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */

package chuckcoughlin.sb.assistant.utilities;

/**
 * Create a model class for various lists - a name/value pair
 */

public class NameValue {
    private String name = "";
    private String value = "";

    public NameValue() {}

    public NameValue(String nam, String val) {
        this.name = nam;
        this.value = val;
    }

    public String getName() {
        return this.name;
    }
    public String getValue() {
        return this.value;
    }
    public void setName(String nam) {
        this.name = nam;
    }
    public void setValue(String val) {
        this.value = val;
    }
}
